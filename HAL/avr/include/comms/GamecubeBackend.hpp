#ifndef _COMMS_GAMECUBEBACKEND_HPP
#define _COMMS_GAMECUBEBACKEND_HPP

#include "core/CommunicationBackend.hpp"
#include "core/state.hpp"

#include <Nintendo.h>

class GamecubeBackend : public CommunicationBackend {
  public:
    GamecubeBackend(
        InputSource **input_sources,
        size_t input_source_count,
        int polling_rate,
        int data_pin
    );
    ~GamecubeBackend();
    void SendReport();

  private:
    CGamecubeConsole *_gamecube;
    Gamecube_Data_t _data;
    int _delay;
    bool _nerfOn;

    // Invoked from the Nintendo library when the final metadata chunk (0xB0
    // sequence) is received. Persists the metadata to EEPROM.
    static void OnMetadataWrite(void *context);
};

#endif
