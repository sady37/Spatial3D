def binary_to_utf8(input_file, output_file):
    """
    将HX711二进制文件转换为UTF-8文本格式
    每行包含: 采样索引, ADC值
    """
    with open(input_file, 'rb') as f:
        raw_bytes = f.read()
    
    # 确保字节数是3的倍数
    if len(raw_bytes) % 3 != 0:
        raw_bytes = raw_bytes[:-(len(raw_bytes) % 3)]
    
    num_samples = len(raw_bytes) // 3
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("# HX711 ADC Data\n")
        f.write("# Format: Sample_Index, ADC_Value\n")
        f.write(f"# Total_Samples: {num_samples}\n")
        f.write("#\n")
        
        for i in range(num_samples):
            start_idx = i * 3
            byte1, byte2, byte3 = raw_bytes[start_idx], raw_bytes[start_idx+1], raw_bytes[start_idx+2]
            
            # 24位二进制补码转32位有符号整数
            raw_24bit = (byte1 << 16) | (byte2 << 8) | byte3
            if raw_24bit & 0x800000:
                adc_value = raw_24bit - 0x1000000
            else:
                adc_value = raw_24bit
            
            f.write(f"{i}, {adc_value}\n")
    
    print(f"转换完成: {input_file} → {output_file}")
    print(f"样本数量: {num_samples}")

# 使用示例
if __name__ == "__main__":
    input_filename = "m5ancbmz6rwx2_1755885601_1755897566677_0.txt"
    output_filename = "m5ancbmz6rwx2_1755885601_1755897566677_0utf8.txt"
    
    binary_to_utf8(input_filename, output_filename)