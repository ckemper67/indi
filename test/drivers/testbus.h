#pragma once

// Do not include INDI driver headers (e.g. ccd_simulator.h) before this file.
// ccd_simulator.h -> dsp.h defines a Log() macro that conflicts with gmock.
// This header owns the gtest include so that ordering is enforced in one place.
#ifdef Log
#  error "dsp.h Log macro detected: include testbus.h before any INDI driver header"
#endif

#include "indiuserio.h"
#include "lilxml.h"
#include "defaultdevice.h"
#include "indipropertynumber.h"
#include "indipropertyswitch.h"
#include "indipropertytext.h"
#include "indipropertylight.h"
#include "indipropertyblob.h"

#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <vector>
#include <cstdarg>
#include <cstring>

// TestBus: in-process replacement for indiserver snoop routing.
//
// Serializes a real driver property to INDI XML using IUUserIO, feeds the bytes
// directly into LilXML, and delivers each complete element to the registered
// consumer devices by calling their ISSnoopDevice() member.  The global
// ISSnoopDevice() function (and its devicesLock) is deliberately NOT used, so:
//   - no lock-inversion deadlock is possible, even with driver background threads
//   - ghost devices (static singletons) never receive stray deliveries
//   - test intent is explicit: only watched consumers receive each delivery
//
// Typical usage:
//   TestScopeSim scope("TestScope");
//   TestCCDSim   ccd;
//   bus.snoop(ccd, scope, [&]{ ccd.setSnoopedTelescope(scope.getDeviceName()); });
//   scope.EqNP[AXIS_RA].setValue(5.5);
//   ASSERT_TRUE(bus.deliver(scope.EqNP));
//   EXPECT_DOUBLE_EQ(ccd.RA, 5.5);
//
// Publisher drivers should override ISSnoopDevice to return false so they do not
// try to process deliveries intended for consumer drivers:
//   bool ISSnoopDevice(XMLEle *) override { return false; }
//
// Threading: deliver() is safe to call while driver background threads are
// running, as long as no driver-internal mutex is held by both the test
// thread's ISSnoopDevice handler AND a background thread that also acquires
// it.  Do not call Connect() on drivers under test if Connect() starts a
// thread that acquires a mutex also used inside ISSnoopDevice.

// Outcome of a single deliver() call.  parsed_messages == 1 is the expected
// normal case; 0 means the XML did not parse or produced no complete element.
// last_xml is capped at 4KB; sufficient for all non-BLOB properties in full,
// and for the XML envelope of BLOB properties (base64 content is truncated).
struct DispatchTrace
{
    int         parsed_messages = 0;
    std::string last_xml;
    std::string parse_error;

    explicit operator bool() const
    {
        return parsed_messages > 0 && parse_error.empty();
    }
};

class TestBus
{
public:
    // Register a consumer device.  Only watched devices receive deliveries.
    // Returns *this so calls can be chained: bus.watch(ccd).watch(guide).
    TestBus &watch(INDI::DefaultDevice &dev)
    {
        consumers_.push_back(&dev);
        return *this;
    }

    // Wire consumer to snoop publisher, then register consumer for deliveries.
    // wire_fn sets up the consumer's device-name configuration (e.g. ActiveDeviceTP,
    // EqNP.setDeviceName) so it accepts messages originating from publisher.
    // Reads as natural INDI vocabulary: "consumer snoops publisher."
    TestBus &snoop(INDI::DefaultDevice &consumer, INDI::DefaultDevice &publisher,
                   std::function<void()> wire_fn)
    {
        (void)publisher;  // name available to caller via publisher.getDeviceName()
        wire_fn();
        return watch(consumer);
    }

    // ---------------------------------------------------------------------------
    // New-style overloads -- pass PropertyNumber/Switch/Text/Light/Blob directly
    // ---------------------------------------------------------------------------
    DispatchTrace deliver(INDI::PropertyNumber p) { return deliver(p->cast()); }
    DispatchTrace deliver(INDI::PropertySwitch p) { return deliver(p->cast()); }
    DispatchTrace deliver(INDI::PropertyText   p) { return deliver(p->cast()); }
    DispatchTrace deliver(INDI::PropertyLight  p) { return deliver(p->cast()); }
    DispatchTrace deliver(INDI::PropertyBlob   p) { return deliver(p->cast()); }

    // ---------------------------------------------------------------------------
    // Raw C-struct overloads
    // ---------------------------------------------------------------------------
    DispatchTrace deliver(const INumberVectorProperty *nvp)
    {
        return stream_deliver([nvp](const userio *io, void *user)
        {
            s_set_number(io, user, nvp);
        });
    }

    DispatchTrace deliver(const ISwitchVectorProperty *svp)
    {
        return stream_deliver([svp](const userio *io, void *user)
        {
            s_set_switch(io, user, svp);
        });
    }

    DispatchTrace deliver(const ITextVectorProperty *tvp)
    {
        return stream_deliver([tvp](const userio *io, void *user)
        {
            s_set_text(io, user, tvp);
        });
    }

    DispatchTrace deliver(const ILightVectorProperty *lvp)
    {
        return stream_deliver([lvp](const userio *io, void *user)
        {
            s_set_light(io, user, lvp);
        });
    }

    DispatchTrace deliver(const IBLOBVectorProperty *bvp)
    {
        return stream_deliver([bvp](const userio *io, void *user)
        {
            s_set_blob(io, user, bvp);
        });
    }

private:
    static constexpr size_t XML_CAP = 4096;

    std::vector<INDI::DefaultDevice *> consumers_;

    struct StreamState
    {
        LilXML                                   *lp;
        const std::vector<INDI::DefaultDevice *> *consumers;
        DispatchTrace                            *trace;
        bool                                      aborted = false;
        char                                      errmsg[256];

        StreamState() : lp(nullptr), consumers(nullptr), trace(nullptr)
        {
            errmsg[0] = '\0';
        }
    };

    // userio write callback: feeds bytes directly into LilXML and dispatches
    // each complete element to consumers.  Tees into trace.last_xml up to
    // XML_CAP bytes so parse errors include enough context for diagnosis.
    static ssize_t s_write(void *user, const void *ptr, size_t n)
    {
        auto *st = static_cast<StreamState *>(user);
        if (st->aborted) return 0;
        const char *p = static_cast<const char *>(ptr);

        std::string &lx = st->trace->last_xml;
        if (lx.size() < XML_CAP)
            lx.append(p, std::min(n, XML_CAP - lx.size()));

        for (size_t i = 0; i < n; ++i)
        {
            XMLEle *root = readXMLEle(st->lp, (unsigned char)p[i], st->errmsg);
            if (root)
            {
                ++st->trace->parsed_messages;
                for (auto *dev : *st->consumers)
                    dev->ISSnoopDevice(root);
                delXMLEle(root);
            }
            else if (st->errmsg[0])
            {
                st->trace->parse_error = st->errmsg;
                ADD_FAILURE() << "TestBus parse error: " << st->errmsg
                              << "\nXML was:\n" << lx;
                st->aborted = true;
                return 0;
            }
        }
        return static_cast<ssize_t>(n);
    }

    // userio vprintf callback: formats into a temporary string, then routes
    // through s_write so the same LilXML feed and tee logic applies.
    // These calls carry only small XML attribute values; the large base64 BLOB
    // content goes through s_write directly via userio_write.
    static int s_vprintf(void *user, const char *fmt, va_list ap)
    {
        va_list ap2;
        va_copy(ap2, ap);
        int needed = vsnprintf(nullptr, 0, fmt, ap2);
        va_end(ap2);
        if (needed < 0) return needed;
        std::string tmp(needed + 1, '\0');
        vsnprintf(tmp.data(), needed + 1, fmt, ap);
        tmp.resize(needed);
        s_write(user, tmp.data(), needed);
        return needed;
    }

    // Variadic trampolines: give IUUserIO a valid va_list while passing
    // fmt=nullptr so the message attribute is omitted entirely.
    static void s_set_number(const userio *io, void *user,
                             const INumberVectorProperty *nvp, ...)
    {
        va_list ap; va_start(ap, nvp);
        IUUserIOSetNumberVA(io, user, nvp, nullptr, ap);
        va_end(ap);
    }

    static void s_set_switch(const userio *io, void *user,
                             const ISwitchVectorProperty *svp, ...)
    {
        va_list ap; va_start(ap, svp);
        IUUserIOSetSwitchVA(io, user, svp, nullptr, ap);
        va_end(ap);
    }

    static void s_set_text(const userio *io, void *user,
                           const ITextVectorProperty *tvp, ...)
    {
        va_list ap; va_start(ap, tvp);
        IUUserIOSetTextVA(io, user, tvp, nullptr, ap);
        va_end(ap);
    }

    static void s_set_light(const userio *io, void *user,
                            const ILightVectorProperty *lvp, ...)
    {
        va_list ap; va_start(ap, lvp);
        IUUserIOSetLightVA(io, user, lvp, nullptr, ap);
        va_end(ap);
    }

    static void s_set_blob(const userio *io, void *user,
                           const IBLOBVectorProperty *bvp, ...)
    {
        va_list ap; va_start(ap, bvp);
        IUUserIOSetBLOBVA(io, user, bvp, nullptr, ap);
        va_end(ap);
    }

    DispatchTrace stream_deliver(std::function<void(const userio *, void *)> fn)
    {
        DispatchTrace trace;
        StreamState st;
        st.lp        = newLilXML();
        st.consumers = &consumers_;
        st.trace     = &trace;

        userio io;
        io.write    = s_write;
        io.vprintf  = s_vprintf;
        io.joinbuff = nullptr;  // BLOBs are base64-encoded inline; no unix socket binary attachment needed

        userio_xmlv1(&io, &st);
        if (!st.aborted)
            fn(&io, &st);

        delLilXML(st.lp);
        return trace;
    }
};
