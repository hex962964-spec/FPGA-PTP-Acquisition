import numpy as np
import matplotlib.pyplot as plt
import struct

# 产生时域下的采样信号，该信号作为IFFT的标准答案
Fs = 512            # 采样频率                   
T = 1/Fs            # 采样周期       
L = 512             # 信号长度
t = np.arange(L)*T  # 时间向量
X_re = 0.0004 + 0.0003*np.sin(2*np.pi*100*t) + 0.0002*np.sin(2*np.pi*200*t)  # 实信号
X_im = np.zeros(L)  # 虚部为零
S = X_re + 1j*X_im  # 复数信号

# 绘制时域信号
plt.figure(1, figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(t, X_re, linewidth=1.5)
plt.xlabel('Time', fontsize=14)
plt.ylabel('Amplitude', fontsize=14)
plt.title('512-point IFFT Standard Result')
plt.grid(True)

# 对时域的采样信号进行快速傅里叶变换，并将结果写入文件，作为电路仿真IFFT的输入
X_fft = np.fft.fft(S, L)  # 快速傅里叶变换

# 频域序列转换为二进制补码并保存
X_fft_re_16bit = np.floor(np.real(X_fft)*(2**15)).astype(np.int16)
X_fft_im_16bit = np.floor(np.imag(X_fft)*(2**15)).astype(np.int16)

# 系统级验证使用C数组灌入数据
with open("ifft512_input.txt", "w") as fd_ifft_input:
    fd_ifft_input.write("const unsigned int fft_input[512] = {\n")
    for i in range(L-1):
        re = struct.unpack("H", struct.pack("h", X_fft_re_16bit[i]))[0]
        im = struct.unpack("H", struct.pack("h", X_fft_im_16bit[i]))[0]
        fd_ifft_input.write(f"0x{re:04x}{im:04x},\r\n")
    
    # 最后一个数据不需要逗号
    re = struct.unpack("H", struct.pack("h", X_fft_re_16bit[L-1]))[0]
    im = struct.unpack("H", struct.pack("h", X_fft_im_16bit[L-1]))[0]
    fd_ifft_input.write(f"0x{re:04x}{im:04x}\r\n")
    fd_ifft_input.write("};\n")

# ifft结果比较
# 读取电路仿真输出的实部和虚部数据
def unsigned2signed(arr, bits):
    """将无符号数转换为有符号数的函数"""
    mask = 2**(bits-1)
    return (arr & (mask-1)) - (arr & mask)

# 读取仿真输出的实部数据
y_re_ifft = np.loadtxt('ifft512_output_re.txt', dtype='str')
y_re_ifft = np.array([int(x, 16) for x in y_re_ifft])

# 读取仿真输出的虚部数据
y_im_ifft = np.loadtxt('ifft512_output_im.txt', dtype='str')
y_im_ifft = np.array([int(x, 16) for x in y_im_ifft])

# 转换为有符号数
y_re_ifft_signed = unsigned2signed(y_re_ifft, 16)/(2**15)
y_im_ifft_signed = unsigned2signed(y_im_ifft, 16)/(2**15)
my_ifft = y_re_ifft_signed + 1j*y_im_ifft_signed

# 绘制电路仿真结果
plt.subplot(2, 1, 2)
plt.plot(t, y_re_ifft_signed, linewidth=1.5)
plt.xlabel('Time', fontsize=14)
plt.ylabel('Amplitude', fontsize=14)
plt.title('512-point FFT Circuit Simulation Result')
plt.grid(True)

# 调整子图间距，防止字符重叠
plt.tight_layout(pad=3.0)

# 计算均值误差
print("平均误差: ", np.mean(np.abs(X_re - y_re_ifft_signed)))

plt.show()

print("信号参数:")
print(f"采样频率: {Fs} Hz")
print(f"采样周期: {T} s")
print(f"信号长度: {L} 点")
print("已生成 ifft512_input.txt 文件，包含C数组格式的输入数据")