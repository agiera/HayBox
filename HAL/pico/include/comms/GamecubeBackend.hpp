#ifndef _COMMS_GAMECUBEBACKEND_HPP
#define _COMMS_GAMECUBEBACKEND_HPP

#include "core/CommunicationBackend.hpp"

#include <GamecubeConsole.hpp>
#include <hardware/pio.h>

class GamecubeBackend : public CommunicationBackend {
  public:
    GamecubeBackend(
        InputSource **input_sources,
        size_t input_source_count,
        uint data_pin,
        bool nerfOn,
        PIO pio = pio0,
        int sm = -1,
        int offset = -1
    );
    ~GamecubeBackend();
    void SendReport();
    int GetOffset();

  private:
    GamecubeConsole *_gamecube;
    gc_report_t _report;
    bool _nerfOn;

    // Invoked from GamecubeConsole when the final metadata chunk (0xB0
    // sequence) is received. Persists the metadata to flash.
    static void OnMetadataWrite(void *context);
};

#endif
