/*
    libastro

    functions used for coordinate conversions, based on libnova

    Copyright (C) 2020 Chris Rowland
    Copyright (C) 2021 Jasem Mutlaq

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation; either version 2.1 of the License, or
    (at your option) any later version.

    This library is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this library; if not, write to the Free Software Foundation,
    Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

*/

#include "libastro.h"
#include "indicom.h"
#include "CoordinateEngine.h"
#include <math.h>
#include <memory>
#include <mutex>

namespace INDI
{

static bool useErfa = false;

void setEngine(bool erfa) {
    useErfa = erfa;
}

ICoordinateEngine& getEngine() {
    static std::unique_ptr<ICoordinateEngine> engine = nullptr;
    static bool lastUseErfa = false;

    if (!engine || useErfa != lastUseErfa) {
        lastUseErfa = useErfa;
        if (useErfa)
            engine = createErfaEngine();
        else
            engine = createLibnovaEngine();
    }
    return *engine;
}

void ObservedToJ2000(IEquatorialCoordinates * observed, double jd, IEquatorialCoordinates * J2000pos)
{
    getEngine().ObservedToJ2000(observed, jd, J2000pos);
}

void J2000toObserved(IEquatorialCoordinates *J2000pos, double jd, IEquatorialCoordinates *observed)
{
    getEngine().J2000toObserved(J2000pos, jd, observed);
}

void EquatorialToHorizontal(IEquatorialCoordinates *object, IGeographicCoordinates *observer, double JD,
                            IHorizontalCoordinates *position)
{
    getEngine().EquatorialToHorizontal(object, observer, JD, position);
}

void GetPlanetObserved(int np, double jd, IEquatorialCoordinates *observed)
{
    getEngine().GetPlanetObserved(np, jd, observed);
}

void HorizontalToEquatorial(IHorizontalCoordinates *object, IGeographicCoordinates *observer, double JD,
                            IEquatorialCoordinates *position)
{
    // FIXME: Implement this for ERFA/EPH engine as well
    ln_lnlat_posn libnova_location = {observer->longitude > 180 ? observer->longitude - 360 : observer->longitude, observer->latitude};
    ln_hrz_posn libnova_object = {range360(object->azimuth + 180), object->altitude};
    ln_equ_posn equatorialPos;
    get_equ_from_hrz(&libnova_object, &libnova_location, JD, &equatorialPos);
    position->rightascension = equatorialPos.ra / 15.0;
    position->declination = equatorialPos.dec;
}

}
