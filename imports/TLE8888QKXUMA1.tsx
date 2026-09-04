import type { ChipProps } from "@tscircuit/props"

const pinLabels = {
  pin1: ["RST"],
  pin2: ["MON"],
  pin3: ["CSN"],
  pin4: ["SDO"],
  pin5: ["SIP"],
  pin6: ["SIN"],
  pin7: ["FCLP"],
  pin8: ["FCLN"],
  pin9: ["T5V1"],
  pin10: ["T5V2"],
  pin11: ["V5V"],
  pin12: ["V6V"],
  pin13: ["VG"],
  pin14: ["OUT7A"],
  pin15: ["OUT7B"],
  pin16: ["OUT7C"],
  pin17: ["OUT20"],
  pin18: ["OUT19"],
  pin19: ["pin19"],
  pin20: ["VDDIO"],
  pin21: ["VROUT"],
  pin22: ["LINTX"],
  pin23: ["LINRX"],
  pin24: ["INJEN"],
  pin25: ["PGND3"],
  pin26: ["KOFFDO"],
  pin27: ["IGNEN"],
  pin28: ["IN1"],
  pin29: ["IN2"],
  pin30: ["IN3"],
  pin31: ["IN4"],
  pin32: ["IN5"],
  pin33: ["IN6"],
  pin34: ["IN7"],
  pin35: ["IN8"],
  pin36: ["IN9"],
  pin37: ["IN10"],
  pin38: ["IN11"],
  pin39: ["IN12"],
  pin40: ["EOTEN"],
  pin41: ["V5VSTBY"],
  pin42: ["CANWKEN"],
  pin43: ["CANRX"],
  pin44: ["CANTX"],
  pin45: ["V5VCAN"],
  pin46: ["CANH"],
  pin47: ["CANL"],
  pin48: ["WK"],
  pin49: ["KEY"],
  pin50: ["PGND2"],
  pin51: ["VRIN2"],
  pin52: ["VRIN1"],
  pin53: ["BATSTBY"],
  pin54: ["BAT"],
  pin55: ["MR"],
  pin56: ["OUT18"],
  pin57: ["OUT17"],
  pin58: ["OUT16"],
  pin59: ["OUT1A"],
  pin60: ["OUT1B"],
  pin61: ["OUT2A"],
  pin62: ["OUT2B"],
  pin63: ["OUT3A"],
  pin64: ["OUT3B"],
  pin65: ["OUT4A"],
  pin66: ["OUT4B"],
  pin67: ["OUT15"],
  pin68: ["OUT14"],
  pin69: ["DFB8"],
  pin70: ["OUT8"],
  pin71: ["DFB9"],
  pin72: ["OUT9"],
  pin73: ["DFB10"],
  pin74: ["OUT10"],
  pin75: ["PGND1"],
  pin76: ["DFB11"],
  pin77: ["OUT11"],
  pin78: ["DFB12"],
  pin79: ["OUT12"],
  pin80: ["DFB13"],
  pin81: ["OUT13"],
  pin82: ["LINIO"],
  pin83: ["OUT5A"],
  pin84: ["OUT5B"],
  pin85: ["OUT5C"],
  pin86: ["OUT24"],
  pin87: ["BATPA"],
  pin88: ["OUT23"],
  pin89: ["OUT22"],
  pin90: ["BATPB"],
  pin91: ["OUT21"],
  pin92: ["OUT6A"],
  pin93: ["OUT6B"],
  pin94: ["OUT6C"],
  pin95: ["CP"],
  pin96: ["IGN1"],
  pin97: ["IGN2"],
  pin98: ["IGN3"],
  pin99: ["IGN4"],
  pin100: ["AGND"],
  pin101: ["pin101"]
} as const

export const TLE8888QKXUMA1 = (props: ChipProps<typeof pinLabels>) => {
  return (
    <chip
      pinLabels={pinLabels}
      supplierPartNumbers={{
  "jlcpcb": [
    "C1518732"
  ]
}}
      manufacturerPartNumber="TLE8888QKXUMA1"
      footprint={<footprint>
        <smtpad portHints={["pin101"]} pcbX="-0mm" pcbY="0mm" width="7.5000104mm" height="7.5000104mm" shape="rect" />
<smtpad portHints={["pin100"]} pcbX="-7.549896mm" pcbY="-5.999988mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin99"]} pcbX="-7.549896mm" pcbY="-5.500116mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin98"]} pcbX="-7.549896mm" pcbY="-4.99999mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin97"]} pcbX="-7.549896mm" pcbY="-4.500118mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin96"]} pcbX="-7.549896mm" pcbY="-3.999992mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin95"]} pcbX="-7.549896mm" pcbY="-3.50012mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin94"]} pcbX="-7.549896mm" pcbY="-2.999994mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin93"]} pcbX="-7.549896mm" pcbY="-2.500122mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin92"]} pcbX="-7.549896mm" pcbY="-1.999996mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin91"]} pcbX="-7.549896mm" pcbY="-1.500124mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin90"]} pcbX="-7.549896mm" pcbY="-0.999998mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin89"]} pcbX="-7.549896mm" pcbY="-0.499872mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin88"]} pcbX="-7.549896mm" pcbY="0mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin87"]} pcbX="-7.549896mm" pcbY="0.500126mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin86"]} pcbX="-7.549896mm" pcbY="0.999998mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin85"]} pcbX="-7.549896mm" pcbY="1.500124mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin84"]} pcbX="-7.549896mm" pcbY="1.999996mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin83"]} pcbX="-7.549896mm" pcbY="2.500122mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin82"]} pcbX="-7.549896mm" pcbY="2.999994mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin81"]} pcbX="-7.549896mm" pcbY="3.50012mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin80"]} pcbX="-7.549896mm" pcbY="3.999992mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin79"]} pcbX="-7.549896mm" pcbY="4.500118mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin78"]} pcbX="-7.549896mm" pcbY="4.99999mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin77"]} pcbX="-7.549896mm" pcbY="5.500116mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin76"]} pcbX="-7.549896mm" pcbY="5.999988mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin75"]} pcbX="-5.999988mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin74"]} pcbX="-5.500116mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin73"]} pcbX="-4.99999mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin72"]} pcbX="-4.500118mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin71"]} pcbX="-3.999992mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin70"]} pcbX="-3.50012mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin69"]} pcbX="-2.999994mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin68"]} pcbX="-2.500122mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin67"]} pcbX="-1.999996mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin66"]} pcbX="-1.500124mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin65"]} pcbX="-0.999998mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin64"]} pcbX="-0.499872mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin63"]} pcbX="-0mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin62"]} pcbX="0.500126mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin61"]} pcbX="0.999998mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin60"]} pcbX="1.500124mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin59"]} pcbX="1.999996mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin58"]} pcbX="2.500122mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin57"]} pcbX="2.999994mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin56"]} pcbX="3.50012mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin55"]} pcbX="3.999992mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin54"]} pcbX="4.500118mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin53"]} pcbX="4.99999mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin52"]} pcbX="5.500116mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin51"]} pcbX="5.999988mm" pcbY="7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin50"]} pcbX="7.549896mm" pcbY="5.999988mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin49"]} pcbX="7.549896mm" pcbY="5.500116mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin48"]} pcbX="7.549896mm" pcbY="4.99999mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin47"]} pcbX="7.549896mm" pcbY="4.500118mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin46"]} pcbX="7.549896mm" pcbY="3.999992mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin45"]} pcbX="7.549896mm" pcbY="3.50012mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin44"]} pcbX="7.549896mm" pcbY="2.999994mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin43"]} pcbX="7.549896mm" pcbY="2.500122mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin42"]} pcbX="7.549896mm" pcbY="1.999996mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin41"]} pcbX="7.549896mm" pcbY="1.500124mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin40"]} pcbX="7.549896mm" pcbY="0.999998mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin39"]} pcbX="7.549896mm" pcbY="0.500126mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin38"]} pcbX="7.549896mm" pcbY="0mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin37"]} pcbX="7.549896mm" pcbY="-0.499872mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin36"]} pcbX="7.549896mm" pcbY="-0.999998mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin35"]} pcbX="7.549896mm" pcbY="-1.500124mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin34"]} pcbX="7.549896mm" pcbY="-1.999996mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin33"]} pcbX="7.549896mm" pcbY="-2.500122mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin32"]} pcbX="7.549896mm" pcbY="-2.999994mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin31"]} pcbX="7.549896mm" pcbY="-3.50012mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin30"]} pcbX="7.549896mm" pcbY="-3.999992mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin29"]} pcbX="7.549896mm" pcbY="-4.500118mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin28"]} pcbX="7.549896mm" pcbY="-4.99999mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin27"]} pcbX="7.549896mm" pcbY="-5.500116mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin26"]} pcbX="7.549896mm" pcbY="-5.999988mm" width="1.5999968mm" height="0.2999994mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin25"]} pcbX="5.999988mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin24"]} pcbX="5.500116mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin23"]} pcbX="4.99999mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin22"]} pcbX="4.500118mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin21"]} pcbX="3.999992mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin20"]} pcbX="3.50012mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin19"]} pcbX="2.999994mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin18"]} pcbX="2.500122mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin17"]} pcbX="1.999996mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin16"]} pcbX="1.500124mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin15"]} pcbX="0.999998mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin14"]} pcbX="0.500126mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin13"]} pcbX="-0mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin12"]} pcbX="-0.499872mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin11"]} pcbX="-0.999998mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin10"]} pcbX="-1.500124mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin9"]} pcbX="-1.999996mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin8"]} pcbX="-2.500122mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin7"]} pcbX="-2.999994mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin6"]} pcbX="-3.50012mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin5"]} pcbX="-3.999992mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin4"]} pcbX="-4.500118mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin3"]} pcbX="-4.99999mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin2"]} pcbX="-5.500116mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<smtpad portHints={["pin1"]} pcbX="-5.999988mm" pcbY="-7.549896mm" width="0.2999994mm" height="1.5999968mm" radius="0.1499997mm" shape="pill" />
<silkscreenpath route={[{"x":6.381140399999936,"y":7.000036800000089},{"x":7.0000113999999485,"y":7.000036800000089}]} />
<silkscreenpath route={[{"x":-6.999986000000149,"y":6.381089599999996},{"x":-6.999986000000149,"y":7.000036800000089},{"x":-6.381115000000136,"y":7.000036800000089}]} />
<silkscreenpath route={[{"x":-6.381115000000136,"y":-6.999986000000035},{"x":-6.999986000000149,"y":-6.999986000000035},{"x":-6.999986000000149,"y":-6.3811657999999625}]} />
<silkscreenpath route={[{"x":7.0000113999999485,"y":-6.3811657999999625},{"x":7.0000113999999485,"y":-6.999986000000035},{"x":6.381140399999936,"y":-6.999986000000035}]} />
<silkscreenpath route={[{"x":7.0000113999999485,"y":7.000036800000089},{"x":7.0000113999999485,"y":6.3811657999999625}]} />
<silkscreenpath route={[{"x":-6.250025600000072,"y":6.250000199999931},{"x":-6.250025600000072,"y":-6.250000199999931},{"x":6.250000199999931,"y":-6.250000199999931},{"x":6.250000199999931,"y":6.250000199999931},{"x":-6.250025600000072,"y":6.250000199999931}]} />
<silkscreenpath route={[{"x":-5.25018,"y":-4.950459999999907},{"x":-5.461114378608045,"y":-4.861251640570117},{"x":-5.547636711767723,"y":-4.649201341500543},{"x":-5.459322117039164,"y":-4.437891200170725},{"x":-5.247640000000047,"y":-4.350471952880866},{"x":-5.035957882960929,"y":-4.437891200170725},{"x":-4.947643288232371,"y":-4.649201341500543},{"x":-5.034165621392276,"y":-4.861251640570117},{"x":-5.245100000000093,"y":-4.950459999999907}]} />
<silkscreenpath route={[{"x":-6.883400000000165,"y":-7.696199999999976},{"x":-7.03214105599784,"y":-7.545557970296159},{"x":-6.882130000000188,"y":-7.396180575985227},{"x":-6.732118944002536,"y":-7.545557970296045},{"x":-6.880860000000098,"y":-7.696199999999976}]} />
<silkscreentext text="{NAME}" pcbX="-0mm" pcbY="9.2042mm" anchorAlignment="center" fontSize="1mm" />
<courtyardoutline outline={[{"x":-8.454200000000128,"y":8.454200000000128},{"x":8.4541999999999,"y":8.454200000000128},{"x":8.4541999999999,"y":-8.936799999999948},{"x":-8.454200000000128,"y":-8.936799999999948},{"x":-8.454200000000128,"y":8.454200000000128}]} />
      </footprint>}
      cadModel={{
        objUrl: "https://modelcdn.tscircuit.com/easyeda_models/assets/C1518732.obj?uuid=8af8f1ba49e244ce9aadc86908fa773a",
        stepUrl: "https://modelcdn.tscircuit.com/easyeda_models/assets/C1518732.step?uuid=8af8f1ba49e244ce9aadc86908fa773a",
        pcbRotationOffset: 0,
        modelOriginPosition: { x: 0, y: 0, z: -1.5 },
      }}
      {...props}
    />
  )
}