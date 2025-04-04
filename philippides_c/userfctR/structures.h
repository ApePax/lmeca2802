#ifndef STRUCTURES_H
#define STRUCTURES_H

//------------------------------------------------------------------------------
// File: structures.h
// Description: Contains all structure definitions of the user functions
//------------------------------------------------------------------------------

#ifdef _WIN32
    // On Windows, we need to export symbols with DLL export attributes
    #define EXPORT_SYMBOL __declspec(dllexport)
#else
    // On Unix-like systems (Linux/macOS), we use the visibility attribute
    #define EXPORT_SYMBOL __attribute__((visibility("default")))
#endif


#define FREQUENCY 50.0
EXPORT_SYMBOL extern const char* filename_CSV;

#endif // STRUCTURES_H
