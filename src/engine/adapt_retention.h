#pragma once

// Memória retida das adaptações (MS42 S20): o aprendido da marcha lenta e os
// totais de misfire por cilindro sobrevivem ao desligamento num AdaptRecord
// com CRC no setor adaptativo da NVM (junto do LTFT/knock).
//
// restore(): no boot, depois de misfire_init(). CRC inválido/ausente → nada
//            é aplicado (marcha lenta parte só do feed-forward, totais a 0).
// save():    com o motor parado (borda rodando→parado no main); o registro
//            só fica dirty se mudou e é gravado pelo flush dos mapas.

namespace ems::engine {

bool adapt_retention_restore() noexcept;
void adapt_retention_save() noexcept;

}  // namespace ems::engine
