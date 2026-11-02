import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

# 设置中文字体支持
# rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']  # 用来正常显示中文标签
# rcParams['axes.unicode_minus'] = False  # 用来正常显示负号

def unsigned2signed(data, bits):
    """将无符号数转换为有符号数"""
    data = np.array(data)
    sign_bit = 2**(bits-1)
    return np.where(data >= sign_bit, data - 2**bits, data)

# 产生时域下的采样信号
Fs = 512            # 采样频率                   
T = 1/Fs            # 采样周期       
L = 512             # 信号长度
t = np.arange(0, L) * T        # 时间向量
X_re = 0.0004 + 0.0003*np.sin(2*np.pi*100*t) + 0.0002*np.sin(2*np.pi*200*t)     # 实信号
X_im = np.zeros(L)
S = X_re + 1j*X_im  # 复信号

# 将采样信号输出至文件，供电路仿真时使用
X_re_16bit = np.floor(X_re * (2**15)).astype(np.int16)  # Q15 格式，表示的范围是：-1<X<0.9999695 
X_im_16bit = np.floor(X_im * (2**15)).astype(np.int16)

# 系统级验证使用C数组灌入数据
with open("fft512_input.txt", 'w') as fd_fft_input:
    fd_fft_input.write('const unsigned int fft_input[512] = {\n')
    
    for i in range(L-1):
        # 将int16转换为uint16表示
        re_uint16 = X_re_16bit[i].view(np.uint16)
        im_uint16 = X_im_16bit[i].view(np.uint16)
        fd_fft_input.write(f'0x{re_uint16:04x}{im_uint16:04x},\r\n')
    
    # 最后一个元素不加逗号
    re_uint16 = X_re_16bit[L-1].view(np.uint16)
    im_uint16 = X_im_16bit[L-1].view(np.uint16)
    fd_fft_input.write(f'0x{re_uint16:04x}{im_uint16:04x}\r\n')
    fd_fft_input.write('};\n')

# 对时域的采样信号进行快速傅里叶变换，画出幅值谱，作为标准结果
X_fft = np.fft.fft(S, L)          # 快速傅里叶变换
X_fft_zero = np.fft.fftshift(X_fft)      # 移动至原点中心对称
fshift = np.arange(-L//2, L//2) * (Fs/L)    # 零中心的频率向量
Amplitude = np.abs(X_fft_zero) * 2 / L    
Amplitude[L//2] = Amplitude[L//2] / 2  # 零中心的幅值谱

plt.figure(1, figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(fshift, Amplitude)
plt.xlabel('Frequency(Hz)', fontsize=14)
plt.ylabel('Amplitude', fontsize=14)
plt.title('512-point FFT Standard Result - Amplitude Spectrum')
plt.grid(True)

# fft结果比较
try:
    # 读取实部数据
    with open('fft512_output_re.txt', 'r') as f1:
        y_re_fft_hex = f1.read().split()
        y_re_fft = [int(x, 16) for x in y_re_fft_hex if x]  # 16进制转换
    
    # 读取虚部数据
    with open('fft512_output_im.txt', 'r') as f2:
        y_im_fft_hex = f2.read().split()
        y_im_fft = [int(x, 16) for x in y_im_fft_hex if x]  # 16进制转换
    
    # 转换为有符号数
    y_re_fft_signed = unsigned2signed(y_re_fft, 16) / (2**15)
    y_im_fft_signed = unsigned2signed(y_im_fft, 16) / (2**15)
    my_fft = y_re_fft_signed + 1j * y_im_fft_signed
    my_fft_zero = np.fft.fftshift(my_fft)      # 移动至原点中心对称
    my_Amplitude = np.abs(my_fft_zero) * 2 / L    
    my_Amplitude[L//2] = my_Amplitude[L//2] / 2  # 零中心的幅值谱
    
    plt.subplot(2, 1, 2)
    plt.plot(fshift, my_Amplitude)
    plt.xlabel('Frequency(Hz)', fontsize=14)
    plt.ylabel('Amplitude', fontsize=14)
    plt.title('512-point FFT Circuit Simulation Result - Amplitude Spectrum')
    plt.grid(True)
    
    # 计算均值误差
    error_mean = np.mean(np.abs(my_Amplitude - Amplitude))
    print(f"平均误差: {error_mean}")
    
except FileNotFoundError as e:
    print(f"文件未找到: {e}")
    print("请确保 'fft512_output_re.txt' 和 'fft512_output_im.txt' 文件存在于当前目录")

plt.tight_layout()
plt.show()

print("信号参数:")
print(f"采样频率: {Fs} Hz")
print(f"采样周期: {T} s")
print(f"信号长度: {L} 点")
print("已生成 fft512_input.txt 文件，包含C数组格式的输入数据")