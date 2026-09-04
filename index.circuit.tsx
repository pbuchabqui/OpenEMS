import React from "react"
import { TLE8888QKXUMA1 } from "./imports/TLE8888QKXUMA1"

// Exploratory tscircuit board for the TLE8888-2QK power/drive hub only
// (docs/hw/schematic/04_tle8888_hub.md + docs/hw/netlist_v1.md, blocks 1/2/4/9/10-11).
// This is NOT the production fab path (that is KiCad, hardware/openems_ecu/,
// see docs/hw/README.md). MCU-side and AMPSEAL J1/J2 connectors are modeled
// as generic 2.54mm headers, not the real WeAct/AMPSEAL footprints.
//
// Scope deliberately excludes: CMP Hall (goes straight J1->MCU, never touches
// the TLE), ETB/BTS7960, analog sensor dividers, USB isolator, VVT beyond the
// two drive channels, Bloco 1 reverse-polarity/fuse/buck/LDO (VBAT arrives
// here already protected, +3V3 arrives already regulated).

export default () => (
  <board width="110mm" height="90mm" layers={4}>
    <schematicsection name="TLE8888" displayName="TLE8888-2QK Hub" />
    <schematicsection name="Power" displayName="Power" />
    <schematicsection name="SPI" displayName="SPI (single-ended)" />
    <schematicsection name="Drive" displayName="INJ / IGN direct drive" />
    <schematicsection name="Aux" displayName="Pump / Fan / VVT" />
    <schematicsection name="CAN" displayName="CAN + CKP digital" />
    <schematicsection name="CKP" displayName="CKP VR front end" />
    <schematicsection name="Connectors" displayName="External connectors" />
    <schematicsection name="SensorRail" displayName="+5V sensor rail out" />

    <TLE8888QKXUMA1
      name="U1"
      schSectionName="TLE8888"
      schX={0}
      schY={0}
      pcbX={0}
      pcbY={0}
      pinAttributes={{
        BAT: { requiresPower: true, mustBeConnected: true },
        VDDIO: { requiresPower: true, mustBeConnected: true },
        AGND: { mustBeConnected: true },
        PGND1: { mustBeConnected: true },
        PGND2: { mustBeConnected: true },
        PGND3: { mustBeConnected: true },
        T5V1: { providesPower: true },
        T5V2: { providesPower: true },
      }}
    />

    {/* ---------------- Power (netlist_v1.md Bloco 1, hub sub-block 4a) ---------------- */}

    <connector
      name="J_PWR_IN"
      schSectionName="Power"
      schX={-16}
      schY={10}
      pcbX={-48}
      pcbY={0}
      footprint="pinrow2_p2.54mm"
      pinLabels={{ pin1: ["V3V3"], pin2: ["AGND_REF"] }}
    />

    <capacitor
      name="C1"
      capacitance="100uF"
      footprint="electrolytic_d6.3mm_p2.5mm"
      schSectionName="Power"
      schOrientation="vertical"
      schX={-12}
      schY={10}
      pcbX={-20}
      pcbY={12}
    />
    <capacitor
      name="C2"
      capacitance="100nF"
      footprint="0402"
      schSectionName="Power"
      schOrientation="vertical"
      schX={-10}
      schY={10}
      pcbX={-24}
      pcbY={18}
    />
    <capacitor
      name="C3"
      capacitance="100nF"
      footprint="0402"
      schSectionName="Power"
      schOrientation="vertical"
      schX={-16}
      schY={8}
      pcbX={-20}
      pcbY={18}
    />
    <capacitor
      name="C4"
      capacitance="100nF"
      footprint="0402"
      schSectionName="Power"
      schOrientation="vertical"
      schX={-14}
      schY={8}
      pcbX={-16}
      pcbY={18}
    />
    {/* VDDIO local decoupling — not called out in the doc but standard IC-supply practice */}
    <capacitor
      name="C5"
      capacitance="100nF"
      footprint="0402"
      schSectionName="Power"
      schOrientation="vertical"
      schX={-12}
      schY={8}
      pcbX={-12}
      pcbY={18}
    />
    {/* CP charge-pump cap: value/topology TBD (Rev 1.2 doesn't give it, see tle8888_pinout.md);
        placeholder pending datasheet confirmation. */}
    <capacitor
      name="C6"
      capacitance="100nF"
      footprint="0402"
      schSectionName="Power"
      schOrientation="vertical"
      schX={-10}
      schY={8}
      pcbX={-8}
      pcbY={18}
    />

    <testpoint
      name="TP_PGND"
      schSectionName="Power"
      schX={-8}
      schY={10}
      pcbX={-24}
      pcbY={4}
      footprintVariant="pad"
      padShape="circle"
      padDiameter="1mm"
    />
    <testpoint
      name="TP_AGND"
      schSectionName="Power"
      schX={-6}
      schY={10}
      pcbX={-28}
      pcbY={4}
      footprintVariant="pad"
      padShape="circle"
      padDiameter="1mm"
    />

    <trace from="U1.BAT" to="net.VBAT" />
    <trace from="U1.BATPA" to="net.VBAT" />
    <trace from="U1.BATPB" to="net.VBAT" />
    <trace from="C1.pin1" to="net.VBAT" />
    <trace from="C1.pin2" to="net.PGND" />
    <trace from="C2.pin1" to="net.VBAT" />
    <trace from="C2.pin2" to="net.PGND" />

    <trace from="U1.PGND1" to="net.PGND" />
    <trace from="U1.PGND2" to="net.PGND" />
    <trace from="U1.PGND3" to="net.PGND" />
    <trace from="U1.thermalpad" to="net.PGND" />
    <trace from="TP_PGND.pin1" to="net.PGND" />

    <trace from="U1.AGND" to="net.AGND" />
    <trace from="TP_AGND.pin1" to="net.AGND" />
    <trace from="J_PWR_IN.AGND_REF" to="net.AGND" />

    <trace from="J_PWR_IN.V3V3" to="net.V3V3" />
    <trace from="U1.VDDIO" to="net.V3V3" />
    <trace from="U1.FCLN" to="net.V3V3" />
    {/* SPI mode strap: FCLN tied to VDDIO/+3V3 selects single-ended mode (tle8888_pinout.md) */}
    <trace from="C5.pin1" to="net.V3V3" />
    <trace from="C5.pin2" to="net.AGND" />

    <trace from="U1.T5V1" to="net.SENS5V_A" />
    <trace from="C3.pin1" to="net.SENS5V_A" />
    <trace from="C3.pin2" to="net.AGND" />

    <trace from="U1.T5V2" to="net.SENS5V_B" />
    <trace from="C4.pin1" to="net.SENS5V_B" />
    <trace from="C4.pin2" to="net.AGND" />

    <trace from="U1.CP" to="C6.pin1" />
    <trace from="C6.pin2" to="net.AGND" />

    {/* +5V sensor rail exposed for the rest of the system (out of scope here) */}
    <connector
      name="J_SENS5V_OUT"
      schSectionName="SensorRail"
      schX={16}
      schY={22}
      pcbX={50}
      pcbY={30}
      pcbRotation={180}
      footprint="pinrow2_p2.54mm"
      pinLabels={{ pin1: ["SENS5V_A"], pin2: ["SENS5V_B"] }}
    />
    <trace from="J_SENS5V_OUT.SENS5V_A" to="net.SENS5V_A" />
    <trace from="J_SENS5V_OUT.SENS5V_B" to="net.SENS5V_B" />

    {/* V5VCAN supply choice is TBD per doc ("V5V do TLE ou rail limpo — ver DS correntes");
        using SENS5V_A here as the "rail limpo" option until current budget is checked. */}
    <trace from="U1.V5VCAN" to="net.SENS5V_A" />

    {/* ---------------- SPI single-ended (hub sub-block 4b) ---------------- */}

    <connector
      name="J_MCU_SPI"
      schSectionName="SPI"
      schX={-16}
      schY={-10}
      pcbX={-48}
      pcbY={-30}
      footprint="pinrow5_p2.54mm"
      pinLabels={{
        pin1: ["SPI_CS"],
        pin2: ["SPI_SCK"],
        pin3: ["SPI_MISO"],
        pin4: ["SPI_MOSI"],
        pin5: ["SPI_GND"],
      }}
    />
    <trace from="J_MCU_SPI.SPI_CS" to="U1.CSN" />
    <trace from="J_MCU_SPI.SPI_SCK" to="U1.FCLP" />
    <trace from="J_MCU_SPI.SPI_MISO" to="U1.SDO" />
    <trace from="J_MCU_SPI.SPI_MOSI" to="U1.SIP" />
    <trace from="J_MCU_SPI.SPI_GND" to="net.AGND" />
    <trace from="U1.SIN" to="net.AGND" />
    {/* SPI mode strap: SIN tied to AGND selects SPI (vs MSC/LVDS) — mandatory per tle8888_pinout.md */}

    {/* ---------------- Direct-drive INJ / IGN (hub sub-block 4c) ---------------- */}

    <connector
      name="J_MCU_DRIVE"
      schSectionName="Drive"
      schX={20}
      schY={-6}
      pcbX={36}
      pcbY={-15}
      pcbRotation={180}
      footprint="pinrow10_p2.54mm"
      pinLabels={{
        pin1: ["INJ1_CMD"],
        pin2: ["INJ2_CMD"],
        pin3: ["INJ3_CMD"],
        pin4: ["INJ4_CMD"],
        pin5: ["INJEN_CMD"],
        pin6: ["IGN1_CMD"],
        pin7: ["IGN2_CMD"],
        pin8: ["IGN3_CMD"],
        pin9: ["IGN4_CMD"],
        pin10: ["IGNEN_CMD"],
      }}
    />
    <trace from="J_MCU_DRIVE.INJ1_CMD" to="U1.IN1" />
    <trace from="J_MCU_DRIVE.INJ2_CMD" to="U1.IN2" />
    <trace from="J_MCU_DRIVE.INJ3_CMD" to="U1.IN3" />
    <trace from="J_MCU_DRIVE.INJ4_CMD" to="U1.IN4" />
    <trace from="J_MCU_DRIVE.INJEN_CMD" to="U1.INJEN" />
    <trace from="J_MCU_DRIVE.IGN1_CMD" to="U1.IN5" />
    <trace from="J_MCU_DRIVE.IGN2_CMD" to="U1.IN6" />
    <trace from="J_MCU_DRIVE.IGN3_CMD" to="U1.IN7" />
    <trace from="J_MCU_DRIVE.IGN4_CMD" to="U1.IN8" />
    <trace from="J_MCU_DRIVE.IGNEN_CMD" to="U1.IGNEN" />

    {/* OUT1..4 A+B must be shorted on copper per datasheet layout rule */}
    <trace from="U1.OUT1A" to="U1.OUT1B" />
    <trace from="U1.OUT1B" to="net.INJ1" />
    <trace from="U1.OUT2A" to="U1.OUT2B" />
    <trace from="U1.OUT2B" to="net.INJ2" />
    <trace from="U1.OUT3A" to="U1.OUT3B" />
    <trace from="U1.OUT3B" to="net.INJ3" />
    <trace from="U1.OUT4A" to="U1.OUT4B" />
    <trace from="U1.OUT4B" to="net.INJ4" />

    <trace from="U1.IGN1" to="net.IGN1" />
    <trace from="U1.IGN2" to="net.IGN2" />
    <trace from="U1.IGN3" to="net.IGN3" />
    <trace from="U1.IGN4" to="net.IGN4" />

    {/* ---------------- Pump / Fan / VVT (hub sub-block 4d) ---------------- */}

    <connector
      name="J_MCU_AUX"
      schSectionName="Aux"
      schX={0}
      schY={-24}
      pcbX={0}
      pcbY={-40}
      pcbRotation={90}
      footprint="pinrow4_p2.54mm"
      pinLabels={{
        pin1: ["PUMP_CMD"],
        pin2: ["FAN_CMD"],
        pin3: ["VVT_EXH_CMD"],
        pin4: ["VVT_INT_CMD"],
      }}
    />
    <trace from="J_MCU_AUX.PUMP_CMD" to="U1.IN9" />
    <trace from="J_MCU_AUX.FAN_CMD" to="U1.IN10" />
    <trace from="J_MCU_AUX.VVT_EXH_CMD" to="U1.IN11" />
    <trace from="J_MCU_AUX.VVT_INT_CMD" to="U1.IN12" />

    <trace from="U1.OUT14" to="net.PUMP_RLY" />
    <trace from="U1.OUT15" to="net.FAN_RLY" />

    {/* OUT5/OUT6 A+B+C must be shorted on copper per datasheet layout rule */}
    <trace from="U1.OUT5A" to="U1.OUT5B" />
    <trace from="U1.OUT5B" to="U1.OUT5C" />
    <trace from="U1.OUT5C" to="net.VVT_EXH" />
    <trace from="U1.OUT6A" to="U1.OUT6B" />
    <trace from="U1.OUT6B" to="U1.OUT6C" />
    <trace from="U1.OUT6C" to="net.VVT_INT" />

    {/* ---------------- CAN + CKP digital (hub sub-blocks 4e / 4f) ---------------- */}

    <connector
      name="J_MCU_MISC"
      schSectionName="CAN"
      schX={0}
      schY={22}
      pcbX={0}
      pcbY={40}
      pcbRotation={270}
      footprint="pinrow3_p2.54mm"
      pinLabels={{
        pin1: ["CAN_TX_FROM_MCU"],
        pin2: ["CAN_RX_TO_MCU"],
        pin3: ["CKP_DIG_TO_MCU"],
      }}
    />
    <trace from="J_MCU_MISC.CAN_TX_FROM_MCU" to="U1.CANTX" />
    <trace from="U1.CANRX" to="J_MCU_MISC.CAN_RX_TO_MCU" />
    <trace from="U1.VROUT" to="J_MCU_MISC.CKP_DIG_TO_MCU" />
    {/* No pull-up on CKP_DIG: VROUT is push-pull (netlist_v1.md Bloco 2). PA0 pull-down lives on
        the MCU side, out of scope for this board. */}

    <resistor
      name="R1"
      resistance="120"
      footprint="0603"
      schSectionName="CAN"
      schX={10}
      schY={18}
      pcbX={30}
      pcbY={38}
    />
    <trace from="R1.pin1" to="net.CANH" />
    <trace from="R1.pin2" to="net.CANL" />
    {/* Bus termination — only stuff this if this board is the physical end of the CAN bus */}

    {/* ---------------- CKP VR front end (netlist_v1.md Bloco 2) ---------------- */}

    <testpoint
      name="TP_VR_P"
      schSectionName="CKP"
      schX={-20}
      schY={-2}
      pcbX={-40}
      pcbY={20}
      footprintVariant="pad"
      padShape="circle"
      padDiameter="1mm"
    />
    <testpoint
      name="TP_VR_N"
      schSectionName="CKP"
      schX={-18}
      schY={-2}
      pcbX={-36}
      pcbY={20}
      footprintVariant="pad"
      padShape="circle"
      padDiameter="1mm"
    />
    <trace from="U1.VRIN1" to="net.CKP_P" />
    <trace from="U1.VRIN2" to="net.CKP_N" />
    <trace from="TP_VR_P.pin1" to="net.CKP_P" />
    <trace from="TP_VR_N.pin1" to="net.CKP_N" />
    {/* No external series R / clamp: the 50 mA input clamp is internal to the TLE8888 */}

    {/* ---------------- External connectors (AMPSEAL J1/J2 — subset relevant to this hub) ---------------- */}

    <connector
      name="J1_SIG"
      schSectionName="Connectors"
      schX={-24}
      schY={2}
      pcbX={-50}
      pcbY={20}
      footprint="pinrow4_p2.54mm"
      pinLabels={{
        pin1: ["CKP_P"],
        pin2: ["CKP_N"],
        pin3: ["CANH"],
        pin4: ["CANL"],
      }}
    />
    <trace from="J1_SIG.CKP_P" to="net.CKP_P" />
    <trace from="J1_SIG.CKP_N" to="net.CKP_N" />
    <trace from="J1_SIG.CANH" to="net.CANH" />
    <trace from="J1_SIG.CANL" to="net.CANL" />
    <trace from="U1.CANH" to="net.CANH" />
    <trace from="U1.CANL" to="net.CANL" />

    <connector
      name="J2_PWR"
      schSectionName="Connectors"
      schX={24}
      schY={2}
      pcbX={34}
      pcbY={8}
      pcbRotation={180}
      footprint="pinrow13_p2.54mm"
      pinLabels={{
        pin1: ["VBAT_IN"],
        pin2: ["PGND_IN"],
        pin3: ["INJ1"],
        pin4: ["INJ2"],
        pin5: ["INJ3"],
        pin6: ["INJ4"],
        pin7: ["IGN1"],
        pin8: ["IGN2"],
        pin9: ["IGN3"],
        pin10: ["IGN4"],
        pin11: ["VVT_EXH"],
        pin12: ["VVT_INT"],
        pin13: ["PUMP_RLY"],
      }}
    />
    {/* FAN_RLY has no free position in this 13-pin subset; wire it directly for now. */}
    <trace from="J2_PWR.VBAT_IN" to="net.VBAT" />
    <trace from="J2_PWR.PGND_IN" to="net.PGND" />
    <trace from="J2_PWR.INJ1" to="net.INJ1" />
    <trace from="J2_PWR.INJ2" to="net.INJ2" />
    <trace from="J2_PWR.INJ3" to="net.INJ3" />
    <trace from="J2_PWR.INJ4" to="net.INJ4" />
    <trace from="J2_PWR.IGN1" to="net.IGN1" />
    <trace from="J2_PWR.IGN2" to="net.IGN2" />
    <trace from="J2_PWR.IGN3" to="net.IGN3" />
    <trace from="J2_PWR.IGN4" to="net.IGN4" />
    <trace from="J2_PWR.VVT_EXH" to="net.VVT_EXH" />
    <trace from="J2_PWR.VVT_INT" to="net.VVT_INT" />
    <trace from="J2_PWR.PUMP_RLY" to="net.PUMP_RLY" />

    <connector
      name="J2_PWR_FAN"
      schSectionName="Connectors"
      schX={24}
      schY={-8}
      pcbX={50}
      pcbY={-5}
      pcbRotation={180}
      footprint="pinrow1_p2.54mm"
      pinLabels={{ pin1: ["FAN_RLY"] }}
    />
    <trace from="J2_PWR_FAN.FAN_RLY" to="net.FAN_RLY" />

    {/* MAIN_RLY (J2 pos 18 / TLE pin 55 MR): reserved, DNP v1 (key-on architecture,
        no MCU power-latch) — intentionally left unwired, no header pin modeled here. */}
  </board>
)
