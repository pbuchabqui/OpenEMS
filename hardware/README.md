# OpenEMS — hardware

Caminho de fabrico da ECU OpenEMS: **microRusEFI adaptado**, não scaffold próprio.

## Estrutura

```
hardware/
├── README.md                 ← este ficheiro
├── openems_ecu/              ← projecto KiCad de produção (editar aqui)
│   ├── openems_ecu.kicad_pro
│   ├── openems_ecu.kicad_sch  ← raiz hierárquica
│   ├── openems_ecu.kicad_pcb
│   ├── mcu_h562.kicad_sch    ← STM32H562VGTx (activo)
│   ├── TLE8888-1QK.kicad_sch
│   ├── adc / hi-lo / pair / Flash / TLE9201 …
│   ├── rusefi_lib/           ← símbolos locais mRE
│   ├── rusefi_lib_external/  ← footprints/libs mRE
│   └── hellen-one → vendor   ← só USB footprint (symlink)
└── vendor/
    └── hw_microRusEfi/       ← submodule git (referência imutável)
```

## Abrir no KiCad 7+

```bash
kicad hardware/openems_ecu/openems_ecu.kicad_pro
```

## Vendor (submodule)

```bash
git submodule update --init hardware/vendor/hw_microRusEfi
cd hardware/vendor/hw_microRusEfi && git submodule update --init --recursive
```

Só consultar / copiar libs. **Não** editar o vendor como board de produção.

## Documentação

| Doc | Papel |
|-----|--------|
| [`docs/hw/microruseefi_as_base.md`](../docs/hw/microruseefi_as_base.md) | Decisão mRE + plano |
| [`docs/hw/netlist_v1.md`](../docs/hw/netlist_v1.md) | O que ligar a quê |
| [`docs/hw/pinout.md`](../docs/hw/pinout.md) | Pinos firmware H562 |
| [`docs/hw/mcu_f407_to_h562_pin_map.md`](../docs/hw/mcu_f407_to_h562_pin_map.md) | 6 pads F407→H562 |
| [`docs/hw/pinmap_logical.md`](../docs/hw/pinmap_logical.md) | Mapa lógico nets |

## O que **não** está aqui (de propósito)

- Scaffold `openems_interface_v1` (removido — não era fabricável)
- Clones avulsos de libs rusEFI/Speeduino sob `hardware/kicad/vendor`
- Hellen-One como base de ECU (só footprint USB via symlink)

## KiCAD MCP (AI assist)

Repo: [mixelpixx/KiCAD-MCP-Server](https://github.com/mixelpixx/KiCAD-MCP-Server.git)  
Install local: `/home/pedro/kicad-mcp-server` (git clone + `npm run build` + venv).

### Config Grok (`~/.grok/config.toml`)

O backend Python **tem** de ser o venv (tem `sexpdata`/`kicad-skip` **e** `pcbnew` via `--system-site-packages`).  
Se usar o `python3` do sistema, o processo Python morre ao arrancar →  
`Python process for KiCAD scripting is not running`.

```toml
[mcp_servers.kicad]
command = "node"
args = ["/home/pedro/kicad-mcp-server/dist/index.js"]
enabled = true
startup_timeout_sec = 60

[mcp_servers.kicad.env]
NODE_ENV = "production"
KICAD_PYTHON = "/home/pedro/kicad-mcp-server/.venv/bin/python"
PYTHONPATH = "/usr/share/kicad/scripting/plugins:/usr/lib/kicad/lib/python3/dist-packages"
LOG_LEVEL = "info"
KICAD_AUTO_LAUNCH = "false"
KICAD_BACKEND = "auto"
```

Reiniciar a sessão Grok após alterar o config. Verificar: `grok mcp list`.

### Actualizar o servidor

```bash
cd /home/pedro/kicad-mcp-server
git pull --ff-only
npm install && npm run build
.venv/bin/pip install -r requirements.txt
```

### Uso com OpenEMS

```bash
# 1) Abrir o projecto no KiCad GUI (opcional para IPC; útil para ver o resultado)
kicad hardware/openems_ecu/openems_ecu.kicad_pro

# 2) No Grok: tools do servidor "kicad"
#    - Esquemático: list_schematic_components, get_schematic_pin_locations, …
#    - PCB: open_board, get_component_pads, route_pad_to_pad, …
```

| Área | Estado neste monorepo |
|------|------------------------|
| Esquemático `mcu_h562.kicad_sch` | OK via MCP (ex.: `list_schematic_components`) |
| PCB `openems_ecu.kicad_pcb` (~4 MB mRE) | `open_board` / `kicad-cli` plot podem **segfault** (SWIG + board grande) |
| Workaround plot | Vistas geradas a partir do `.kicad_pcb` (ex. PIL) se o plot falhar |

Diagnóstico rápido do backend:

```bash
# Tem de imprimir {"type":"ready"} e NÃO ModuleNotFoundError: sexpdata
echo '{}' | /home/pedro/kicad-mcp-server/.venv/bin/python \
  /home/pedro/kicad-mcp-server/python/kicad_interface.py
```

Sucessor do projecto (opcional): [Konnect](https://github.com/mixelpixx/Konnect) — plugin Rust nativo KiCad 10 / IPC.

## Créditos

Hardware base © rusEFI / microRusEFI. OpenEMS adapta com atribuição.
