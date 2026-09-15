/**
 * station.js - Moss Landing station geometry, shared by the phone and the shore view.
 *
 * Every constant here is MEASURED and mirrors the Console source. Do not "improve" a number
 * without changing it in services/vision/optics.ts or georef.ts first; a beacon that disagrees
 * with the Console about where the water is will be confidently wrong on both screens.
 *
 *   station position + height   services/vision/optics.ts (cameraHeightM 8.40)
 *   field of regard 208-332     OPTICS.forStartDeg / forEndDeg
 *   RGB zoom -> focal ladder    measured on the installed head 2026-08-30
 *   boresight offset 111.69     pan = trueBearing - 111.69  (pan is NOT bearing)
 *   sun exclusion 5 deg         ptzAiming.ts checkAimSafety, sunAvoidDeg
 */

export const STATION = { lat: 36.8034223, lon: -121.788027, heightM: 8.40 };
export const FOR_START = 208, FOR_END = 332;
export const BORESIGHT_DEG = 111.69;
export const SUN_AVOID_DEG = 5;
export const ZOOM_MAX = 32;
export const TOTAL_LAG_S = 6;          // phone post + relay + console poll + slew
export const SUBJECT_HEIGHT_M = 1.7;   // a standing person; a hull reads larger, never smaller

/** RGB zoom -> focal length in pixels. Measured, not the old linear model. */
const FOCAL_LADDER = [
    [1, 1665], [7.2, 12066], [10.3, 17743], [13.4, 23750], [16.5, 29787],
    [19.6, 36022], [22.7, 45323], [25.8, 52770], [28.89, 60556], [32, 70784],
];

const R_EARTH_KM = 6371.0088, R_EARTH_M = 6371000;
const RAD = Math.PI / 180, DEG = 180 / Math.PI;

export function focalPxAtZoom(zoom) {
    const z = Math.min(Math.max(zoom, 1), ZOOM_MAX);
    for (let i = 1; i < FOCAL_LADDER.length; i++) {
        const a = FOCAL_LADDER[i - 1], b = FOCAL_LADDER[i];
        if (z <= b[0]) return a[1] + (b[1] - a[1]) * (z - a[0]) / (b[0] - a[0]);
    }
    return FOCAL_LADDER[FOCAL_LADDER.length - 1][1];
}
export const hfovDeg = (z) => 2 * Math.atan(960 / focalPxAtZoom(z)) * DEG;
export const vfovDeg = (z) => 2 * Math.atan(540 / focalPxAtZoom(z)) * DEG;

export function initialBearingDeg(lat1, lon1, lat2, lon2) {
    const p1 = lat1 * RAD, p2 = lat2 * RAD, dl = (lon2 - lon1) * RAD;
    const y = Math.sin(dl) * Math.cos(p2);
    const x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
    return (Math.atan2(y, x) * DEG + 360) % 360;
}
export function greatCircleKm(lat1, lon1, lat2, lon2) {
    const p1 = lat1 * RAD, p2 = lat2 * RAD;
    const dp = (lat2 - lat1) * RAD, dl = (lon2 - lon1) * RAD;
    const a = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
    return R_EARTH_KM * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}
export const inFieldOfRegard = (b) => b >= FOR_START && b <= FOR_END;
export const panForBearing = (b) => (b - BORESIGHT_DEG + 360) % 360;
export const horizonKm = Math.sqrt(2 * R_EARTH_M * STATION.heightM + STATION.heightM ** 2) / 1000;

/**
 * THE ZOOM LAW. Tighten until the position error fills about a third of the frame, then stop.
 *
 * georef.aimAt caps magnification at 8x because it was written for stale AIS on a moving hull.
 * A beacon reports its OWN accuracy, speed and age, so it can do better - and must, because the
 * naive answer is wrong in a direction that loses the target. At 220 m an 8 m GPS error is 2.1
 * degrees of bearing, wider than the whole 1.55 degree field at 32x: max zoom would put the
 * operator reliably OUTSIDE the frame. Close in, the binding constraint is GPS, not the lens.
 */
export function solveAim(fix, opts) {
    const lagS = (opts && opts.lagS != null) ? opts.lagS : TOTAL_LAG_S;
    const bearingDeg = initialBearingDeg(STATION.lat, STATION.lon, fix.lat, fix.lon);
    const rangeKm = greatCircleKm(STATION.lat, STATION.lon, fix.lat, fix.lon);
    const rangeM = Math.max(rangeKm * 1000, 25);

    const leadM = Math.max(0, fix.speed || 0) * lagS;
    const accuracyM = fix.accuracy != null ? fix.accuracy : 10;
    const errorM = Math.sqrt(accuracyM * accuracyM + leadM * leadM);

    const neededHfov = 3 * (2 * Math.atan(errorM / rangeM) * DEG);
    let zoomX = 1;
    for (let z = ZOOM_MAX; z >= 1; z -= 0.1) { if (hfovDeg(z) >= neededHfov) { zoomX = z; break; } }
    zoomX = Math.min(ZOOM_MAX, Math.max(1, zoomX));

    const subjectPx = Math.atan(SUBJECT_HEIGHT_M / rangeM) * DEG / vfovDeg(zoomX) * 1080;
    const tiltDeg = -(STATION.heightM / rangeM + rangeM / (2 * R_EARTH_M)) * DEG;

    return {
        bearingDeg, rangeKm, rangeMi: rangeKm * 0.621371, tiltDeg, zoomX, subjectPx,
        errorM, leadM, accuracyM,
        panDeg: panForBearing(bearingDeg),
        inArc: inFieldOfRegard(bearingDeg),
        beyondHorizon: rangeKm >= horizonKm,
        /* Under ~24 px the head will hold you but the footage is not worth much. */
        resolvable: subjectPx >= 24,
    };
}

/**
 * Dead reckoning between fixes. Advance the last fix along its own reported course.
 * The result is ALWAYS labelled deadReckoned - a reckoned position is not a fix, and the
 * distinction has to survive all the way into the record. (No-fake-claims rule 6.)
 */
export function deadReckon(fix, atMs) {
    const dtS = (atMs - fix.t) / 1000;
    if (!(dtS > 0) || !fix.speed || fix.heading == null) {
        return Object.assign({}, fix, { deadReckoned: false });
    }
    const distM = fix.speed * dtS;
    const brg = fix.heading * RAD;
    const dLat = (distM * Math.cos(brg)) / 111320;
    const dLon = (distM * Math.sin(brg)) / (111320 * Math.cos(fix.lat * RAD));
    return Object.assign({}, fix, {
        lat: fix.lat + dLat, lon: fix.lon + dLon,
        accuracy: (fix.accuracy || 10) + distM * 0.25,   // reckoning degrades the estimate
        deadReckoned: true, reckonedForS: dtS,
    });
}

/**
 * Solar position, low-precision NOAA approximation. Good to a fraction of a degree, which is
 * plenty against a 5 degree exclusion cone. The Console owns the authoritative sun model; this
 * exists so the phone can warn you BEFORE you paddle into a bearing the station will refuse.
 */
export function sunPosition(date) {
    const d = date || new Date();
    const start = Date.UTC(d.getUTCFullYear(), 0, 0);
    const dayOfYear = Math.floor((d.getTime() - start) / 86400000);
    const decl = 23.44 * RAD * Math.sin(2 * Math.PI * (284 + dayOfYear) / 365);
    const B = 2 * Math.PI * (dayOfYear - 81) / 364;
    const eot = 9.87 * Math.sin(2 * B) - 7.53 * Math.cos(B) - 1.5 * Math.sin(B);
    const utcHours = d.getUTCHours() + d.getUTCMinutes() / 60 + d.getUTCSeconds() / 3600;
    const solarTime = utcHours + STATION.lon / 15 + eot / 60;
    const H = (solarTime - 12) * 15 * RAD;
    const phi = STATION.lat * RAD;
    const elevationDeg = Math.asin(
        Math.sin(phi) * Math.sin(decl) + Math.cos(phi) * Math.cos(decl) * Math.cos(H)) * DEG;
    let azimuthDeg = Math.atan2(
        Math.sin(H), Math.cos(H) * Math.sin(phi) - Math.tan(decl) * Math.cos(phi)) * DEG + 180;
    azimuthDeg = (azimuthDeg + 360) % 360;
    return { azimuthDeg, elevationDeg, up: elevationDeg > -0.833 };
}

/** Angular separation between an aim and the sun. Below SUN_AVOID_DEG the Console hard-denies. */
export function sunConflict(bearingDeg, tiltDeg, at) {
    const sun = sunPosition(at);
    if (!sun.up) return { conflict: false, separationDeg: null, sun };
    const dAz = Math.abs(((bearingDeg - sun.azimuthDeg + 540) % 360) - 180);
    const dEl = Math.abs((tiltDeg || 0) - sun.elevationDeg);
    const separationDeg = Math.sqrt(dAz * dAz + dEl * dEl);
    return { conflict: separationDeg < SUN_AVOID_DEG, separationDeg, sun };
}
