# OpenEMS — KiCad hardware

## Caminho de fabrico (activo)

**Projecto:** [`../openems_ecu/openems_ecu.kicad_pro`](../openems_ecu/openems_ecu.kicad_pro)  
**Base:** microRusEFI (`hardware/vendor/hw_microRusEfi`)  
**Arquitectura:** [`../../docs/hw/microruseefi_as_base.md`](../../docs/hw/microruseefi_as_base.md)

```bash
kicad hardware/openems_ecu/openems_ecu.kicad_pro
```

| Area | State |
|------|--------|
| Cópia de trabalho a partir do mRE | ✅ |
| TLE8888 no esquemático/PCB vendor | ✅ (auditar) |
| MCU H562 soldado | ❌ sheet a criar |
| Conector / ETB OpenEMS | ⏳ decisões abertas |
| Fab | ❌ só após adaptação + DRC |

## Scaffold experimental (NÃO fabricar)

**Projecto:** `openems_interface_v1/` — útil para pinmap/docs e protótipo de hierarquia.  
**Não** é o caminho de fabrico. Ver `openems_interface_v1/LEIA-ME.md`.

Scripts `layout_clean.py` / `build_all.sh` **não** geram a PCB de produção.

## Hellen-One

**Não** usado como base da ECU OpenEMS. Módulos oficiais não têm TLE8888 nem H562.  
Pode voltar a ser considerado mais tarde só para adaptadores PnP por veículo.
