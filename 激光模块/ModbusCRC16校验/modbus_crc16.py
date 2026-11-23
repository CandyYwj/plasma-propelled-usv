def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001  # 0xA001 是 0x8005 的反序
            else:
                crc >>= 1
    return crc


def hexstr_to_bytes(hexstr: str) -> bytes:
    return bytes(int(x, 16) for x in hexstr.strip().split())


def main():
    hex_input = input("Input data (e.g. A5 03 20 01 00 00 00): ")
    data = hexstr_to_bytes(hex_input)
    crc = modbus_crc16(data)
    # 低字节在前，高字节在后
    crc_bytes = crc.to_bytes(2, byteorder='little')
    result = data + crc_bytes
    print("Output:", ' '.join(f"{b:02X}" for b in result))


if __name__ == "__main__":
    main() 