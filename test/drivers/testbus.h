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
// Serializes a real driver property to INDI XML using IUUserIO, parses the
// result with LilXML, and delivers it directly to the registered consumer
// devices by calling their ISSnoopDevice() member.  The global ISSnoopDevice()
// function (and its devicesLock) is deliberately NOT used, so:
//   - no lock-inversion deadlock is possible, even with driver background threads
//   - ghost devices (static singletons) never receive stray deliveries
//   - test intent is explicit: only watched consumers receive each delivery
//
// Typical usage:
//   MockScopeDriver scope("MockScope");
//   MockCCDDriver   ccd;
//   ccd.setSnoopedTelescope("MockScope");
//   bus.watch(ccd);
//   scope.EqNP[AXIS_RA].setValue(5.5);
//   auto t = bus.deliver(scope.EqNP);
//   ASSERT_EQ(t.parsed_messages, 1) << t.parse_error;
//   EXPECT_DOUBLE_EQ(ccd.RA, 5.5);
//
// Publisher mocks should override ISSnoopDevice to return false so they do not
// try to process deliveries intended for consumer mocks:
//   bool ISSnoopDevice(XMLEle *) override { return false; }
//
// Threading: deliver() is safe to call while driver background threads are
// running, as long as no driver-internal mutex is held by both the test
// thread's ISSnoopDevice handler AND a background thread that also acquires
// it.  Do not call Connect() on drivers under test if Connect() starts a
// thread that acquires a mutex also used inside ISSnoopDevice.

// Outcome of a single deliver() call.  parsed_messages == 1 is the expected
// normal case; 0 means the XML did not parse or produced no complete element.
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
        return dispatch_xml(capture([nvp](const userio *io, void *user)
        {
            s_set_number(io, user, nvp);
        }));
    }

    DispatchTrace deliver(const ISwitchVectorProperty *svp)
    {
        return dispatch_xml(capture([svp](const userio *io, void *user)
        {
            s_set_switch(io, user, svp);
        }));
    }

    DispatchTrace deliver(const ITextVectorProperty *tvp)
    {
        return dispatch_xml(capture([tvp](const userio *io, void *user)
        {
            s_set_text(io, user, tvp);
        }));
    }

    DispatchTrace deliver(const ILightVectorProperty *lvp)
    {
        return dispatch_xml(capture([lvp](const userio *io, void *user)
        {
            s_set_light(io, user, lvp);
        }));
    }

    DispatchTrace deliver(const IBLOBVectorProperty *bvp)
    {
        return dispatch_xml(capture([bvp](const userio *io, void *user)
        {
            s_set_blob(io, user, bvp);
        }));
    }

private:
    std::vector<INDI::DefaultDevice *> consumers_;

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

    // userio callbacks that append to a std::string buffer
    static ssize_t s_write(void *user, const void *ptr, size_t n)
    {
        static_cast<std::string *>(user)->append(static_cast<const char *>(ptr), n);
        return static_cast<ssize_t>(n);
    }

    static int s_vprintf(void *user, const char *fmt, va_list ap)
    {
        va_list ap2;
        va_copy(ap2, ap);
        int needed = vsnprintf(nullptr, 0, fmt, ap2);
        va_end(ap2);
        if (needed < 0) return needed;
        std::string &s = *static_cast<std::string *>(user);
        size_t old = s.size();
        s.resize(old + needed + 1);
        vsnprintf(s.data() + old, needed + 1, fmt, ap);
        s.resize(old + needed);
        return needed;
    }

    std::string capture(std::function<void(const userio *, void *)> fn)
    {
        std::string buf;
        userio io;
        io.write    = s_write;
        io.vprintf  = s_vprintf;
        io.joinbuff = nullptr;
        userio_xmlv1(&io, &buf);
        fn(&io, &buf);
        return buf;
    }

    DispatchTrace dispatch_xml(const std::string &xml)
    {
        DispatchTrace trace;
        trace.last_xml = xml;
        LilXML *lp = newLilXML();
        char errmsg[256] = {0};
        for (char c : xml)
        {
            XMLEle *root = readXMLEle(lp, c, errmsg);
            if (root)
            {
                ++trace.parsed_messages;
                for (auto *dev : consumers_)
                    dev->ISSnoopDevice(root);
                delXMLEle(root);
            }
            else if (errmsg[0])
            {
                trace.parse_error = errmsg;
                ADD_FAILURE() << "TestBus parse error: " << errmsg
                              << "\nXML was:\n" << xml;
                break;
            }
        }
        delLilXML(lp);
        return trace;
    }
};
