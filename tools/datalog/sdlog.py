#!/usr/bin/env python3
"""Datalog do OpenEMS em cartão SD: preparar, decodificar e autoteste.

Formato (docs/datalog.md): blocos brutos de 512 B a partir do setor 2048.
Cada bloco = cabeçalho de 16 B (magic "OLG1", sessão, sequência, CRC-32 do
bloco com o campo CRC zerado) + 15 registros de 32 B + 16 B reservados.

    sdlog.py format <dispositivo|imagem> [--blocks N]
    sdlog.py decode <dispositivo|imagem> [-o saida.csv]
    sdlog.py selftest
"""
import argparse
import csv
import io
import struct
import sys
import zlib

BLOCK = 512
START_LBA = 2048
REGION_BLOCKS = 262144
MAGIC = 0x31474C4F  # "OLG1"
HEADER = 16
RECORD = 32
RECORDS_PER_BLOCK = 15

# t_ms u32, rpm u16, map u16, tps u16, clt s16, iat s16, lambda u16, alvo u16,
# pw u16, avanço s16, stft s16, ltft s16, status u16, cortes u16, sync u8, flags u8
RECORD_FMT = "<IHHHhhHHHhhhHHBB"
assert struct.calcsize(RECORD_FMT) == RECORD

COLUMNS = [
    "session", "seq", "t_ms", "rpm", "map_kpa", "tps_pct", "clt_c", "iat_c",
    "lambda", "lambda_target", "pw_ms", "advance_deg", "stft_pct", "ltft_pct",
    "status_bits", "cut_reasons", "sync_state", "flags",
]


def block_crc(block: bytes) -> int:
    return zlib.crc32(block[:12] + b"\0\0\0\0" + block[16:]) & 0xFFFFFFFF


def make_block(session: int, seq: int, records) -> bytes:
    assert len(records) == RECORDS_PER_BLOCK
    body = b"".join(struct.pack(RECORD_FMT, *r) for r in records)
    block = struct.pack("<IIII", MAGIC, session, seq, 0) + body
    block += b"\0" * (BLOCK - len(block))
    return block[:12] + struct.pack("<I", block_crc(block)) + block[16:]


def parse_block(block: bytes):
    """Devolve (session, seq, [registros]) ou None se o bloco for inválido."""
    if len(block) != BLOCK:
        return None
    magic, session, seq, crc = struct.unpack_from("<IIII", block)
    if magic != MAGIC or crc != block_crc(block):
        return None
    recs = [
        struct.unpack_from(RECORD_FMT, block, HEADER + i * RECORD)
        for i in range(RECORDS_PER_BLOCK)
    ]
    return session, seq, recs


def row(session, seq, r):
    (t, rpm, mp, tps, clt, iat, lam, lamt, pw, adv, stft, ltft, st, cut, sync, fl) = r
    return [
        session, seq, t, rpm / 10, mp, tps / 10, clt / 10, iat / 10,
        lam / 1000, lamt / 1000, pw / 1000, adv / 10, stft / 10, ltft / 10,
        st, cut, sync, fl,
    ]


def decode(f, out):
    """Lê os blocos válidos em sequência e escreve o CSV. Devolve (blocos, lacunas)."""
    w = csv.writer(out)
    w.writerow(COLUMNS)
    f.seek(START_LBA * BLOCK)
    blocks = gaps = 0
    last = {}
    for _ in range(REGION_BLOCKS):
        parsed = parse_block(f.read(BLOCK))
        if parsed is None:
            break
        session, seq, recs = parsed
        if session in last and seq != last[session] + 1:
            gaps += seq - last[session] - 1
        last[session] = seq
        for r in recs:
            w.writerow(row(session, seq, r))
        blocks += 1
    return blocks, gaps


def cmd_format(args):
    zero = b"\0" * BLOCK
    with open(args.device, "r+b") as f:
        f.seek(START_LBA * BLOCK)
        for _ in range(args.blocks):
            f.write(zero)
    print(f"{args.blocks} blocos zerados a partir do setor {START_LBA}")


def cmd_decode(args):
    with open(args.device, "rb") as f:
        if args.output:
            with open(args.output, "w", newline="") as out:
                blocks, gaps = decode(f, out)
        else:
            blocks, gaps = decode(f, sys.stdout)
    print(f"{blocks} blocos, {gaps} blocos perdidos (lacunas de sequência)", file=sys.stderr)


# Registro dourado: o mesmo vetor está em test/test_datalog.cpp.
GOLDEN_SAMPLE = (1000, 8000, 45, 120, -50, 321, 1002, 1000, 3456, -25, 17, -9, 0x0241, 4, 3, 1)
GOLDEN_BYTES = bytes([
    0xE8, 0x03, 0x00, 0x00, 0x40, 0x1F, 0x2D, 0x00, 0x78, 0x00, 0xCE, 0xFF,
    0x41, 0x01, 0xEA, 0x03, 0xE8, 0x03, 0x80, 0x0D, 0xE7, 0xFF, 0x11, 0x00,
    0xF7, 0xFF, 0x41, 0x02, 0x04, 0x00, 0x03, 0x01,
])


def cmd_selftest(_args):
    assert struct.pack(RECORD_FMT, *GOLDEN_SAMPLE) == GOLDEN_BYTES, "registro dourado difere do firmware"

    image = bytearray(b"\0" * (START_LBA * BLOCK))
    seqs = [(1, 0), (1, 1), (1, 3), (2, 0)]          # lacuna de 1 bloco na sessão 1
    for session, seq in seqs:
        recs = [GOLDEN_SAMPLE] * RECORDS_PER_BLOCK
        image += make_block(session, seq, recs)
    image += b"\0" * BLOCK                             # fim do log

    out = io.StringIO()
    blocks, gaps = decode(io.BytesIO(bytes(image)), out)
    assert blocks == 4, blocks
    assert gaps == 1, gaps
    rows = list(csv.reader(io.StringIO(out.getvalue())))
    assert rows[0] == COLUMNS
    assert len(rows) == 1 + 4 * RECORDS_PER_BLOCK
    first = rows[1]
    assert first[2] == "1000" and first[3] == "800.0" and first[6] == "-5.0", first
    assert first[10] == "3.456" and first[11] == "-2.5", first

    corrupt = bytearray(make_block(1, 0, [GOLDEN_SAMPLE] * RECORDS_PER_BLOCK))
    corrupt[100] ^= 1
    assert parse_block(bytes(corrupt)) is None, "bloco com CRC errado foi aceito"
    print("sdlog selftest: OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("format")
    p.add_argument("device")
    p.add_argument("--blocks", type=int, default=REGION_BLOCKS)
    p.set_defaults(fn=cmd_format)
    p = sub.add_parser("decode")
    p.add_argument("device")
    p.add_argument("-o", "--output")
    p.set_defaults(fn=cmd_decode)
    p = sub.add_parser("selftest")
    p.set_defaults(fn=cmd_selftest)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
