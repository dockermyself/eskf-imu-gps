#ifndef _NMEA_H_
#define _NMEA_H_

//GGA
enum {
    kGGATime = 1,
    kGGALatitude = 2,
    kGGANorS = 3,
    kGGALongitude = 4,
    kGGAEorW = 5,
    kGGAQuality = 6,
    kGGASatelliteNum = 7,
    kGGAHdop = 8,
    kGGAAltitude = 9,
    kGGAEllipsoideHeight = 11,
    kGGADiffTime = 13,
    kGGADiffStationID = 14,
    kGGACheckSum = 15,
};

//RMC
enum {
    kRMCTime = 1,
    kRMCStatus = 2,
    kRMCLatitude = 3,
    kRMCNorS = 4,
    kRMCLongitude = 5,
    kRMCEorW = 6,
    kRMCSpeed = 7,
    kRMCCourse = 8,
    kRMCData = 9,
    kRMCMagneticVariation = 10,
    kRMCMagneticVariationEW = 11,
    kRMCModeIndition = 12,
    kRMCCheckSum = 13,
};

#endif