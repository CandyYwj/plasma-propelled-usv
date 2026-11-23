def reflect_byte(byte_val: int) -> int:
    """Reflects the bits of an 8-bit byte."""
    reflected_byte = 0
    for i in range(8):
        if (byte_val >> i) & 1:
            reflected_byte |= (1 << (7 - i))
    return reflected_byte

def reflect_word(word_val: int) -> int:
    """Reflects the bits of a 16-bit word."""
    reflected_word = 0
    for i in range(16):
        if (word_val >> i) & 1:
            reflected_word |= (1 << (15 - i))
    return reflected_word

def calculate_modbus_crc16(data: bytes) -> bytes:
    """
    Calculates the Modbus CRC16 checksum for a given byte sequence.
    This implementation includes input byte reflection and final CRC reflection.

    Args:
        data: The byte sequence to calculate the CRC for.

    Returns:
        A 2-byte sequence representing the CRC16 checksum,
        formatted as [LSB_of_CRC_value, MSB_of_CRC_value]
        to match your frame's required byte order (0F 0B for the example).
    """
    # Standard Modbus CRC16 implementation with reflected input/output
    crc = 0xFFFF
    polynomial = 0xA001 # Reflected polynomial of 0x8005

    for byte_val in data:
        crc ^= byte_val
        for _ in range(8):
            if crc & 0x0001: # Check the LSB of the current CRC value
                crc = (crc >> 1) ^ polynomial
            else:
                crc >>= 1

    # The calculation result (crc) is a 16-bit integer (e.g., 0x0B0F).
    # According to your analysis, the frame stores this as LSB first, then MSB (0x0F 0x0B).
    # We need to explicitly construct the bytes object with LSB followed by MSB.

    # Extract LSB and MSB from the calculated 16-bit CRC
    lsb = crc & 0x00FF  # Get the low 8 bits (0x0F for 0x0B0F)
    msb = (crc >> 8) & 0x00FF # Get the high 8 bits (0x0B for 0x0B0F)

    # Construct the bytes object in the order LSB, MSB
    # This should produce the desired 0F 0B byte sequence for the example 0x0B0F
    return bytes([lsb, msb])


if __name__ == "__main__":
    # 示例数据 (来自你的第一个验证成功的数据帧)
    # 数据部分: A5 03 20 01 00 00 0E FF FF FF FF FF FF 7A 00 52 00 98 2E 01 00
    # 期望CRC (高字节在前, 低字节在后): FC 9D
    # 标准Modbus计算结果: 0x9DFC (LSB=FC, MSB=9D) -> 在帧中应为 FC 9D (按你的高低字节描述)

    # 示例数据 (来自你的第二个验证成功的版本查询帧)
    # 数据部分: A5 03 20 0A 00 00 12 44 54 53 36 30 31 32 5F 41 50 50 5F 56 31 2E 32 36 43
    # 期望CRC (高字节在前, 低字节在后): 0F 0B
    # 标准Modbus计算结果: 0x0B0F (LSB=0F, MSB=0B) -> 在帧中应为 0F 0B (按你的高低字节描述)

    data_hex_string_with_spaces = input("请输入应答帧的数据部分（不包含CRC，十六进制格式，可以有空格，例如 A5 03 20 01...）：")
    data_hex_string_without_spaces = data_hex_string_with_spaces.replace(" ", "")

    try:
        if not data_hex_string_without_spaces:
            print("输入数据为空，无法计算CRC。")
        else:
            data_bytes = bytes.fromhex(data_hex_string_without_spaces)
            print(f"输入数据 (bytes): {data_bytes.hex().upper()}")

            # Call the CRC calculation function
            calculated_crc_bytes_from_function = calculate_modbus_crc16(data_bytes)

            # Print the raw bytes returned by the function (which is currently MSB LSB order)
            print(f"calculate_modbus_crc16 返回的原始 bytes (当前 MSB LSB 顺序): {calculated_crc_bytes_from_function}")

            # Explicitly swap the bytes to get the required LSB MSB order for the frame
            if len(calculated_crc_bytes_from_function) == 2:
                # Construct the new bytes object with the bytes swapped
                calculated_crc_bytes_for_frame = bytes([calculated_crc_bytes_from_function[1], calculated_crc_bytes_from_function[0]])
            else:
                # Handle unexpected length, though the function should always return 2 bytes
                print("错误：CRC 计算函数返回的字节长度不正确。")
                calculated_crc_bytes_for_frame = b'' # Use empty bytes to avoid further errors

            # Print the CRC bytes after swapping, in the required LSB MSB order
            print(f"计算得到的CRC (用于帧, 交换后为 LSB MSB 顺序): {calculated_crc_bytes_for_frame.hex().upper()}")

            # Build the full response frame by appending the swapped CRC bytes
            full_response_frame = data_bytes + calculated_crc_bytes_for_frame

            # Output the full response frame
            full_response_hex_with_spaces = ' '.join([full_response_frame.hex().upper()[i:i+2] for i in range(0, len(full_response_frame.hex()), 2)])
            print(f"完整应答帧 (含CRC): {full_response_hex_with_spaces}")

    except ValueError as e:
        print(f"输入错误：请确保输入的是有效的十六进制字符。详情: {e}")
    except Exception as e:
        print(f"发生未知错误: {e}")