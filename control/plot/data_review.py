import re
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from itertools import islice

def extract_numbers_with_regex(text):
    """使用正则表达式提取lat和lon数值"""
    try:
        # 匹配lat和lon后的数字（支持正负浮点数）
        pattern = r'(?i)lat\s*[:=]\s*(-?\d+(?:\.\d+)?)\s*,\s*lon\s*[:=]\s*(-?\d+(?:\.\d+)?)'
        match = re.search(pattern, text)
        
        if match:
            lat = float(match.group(1))
            lon = float(match.group(2))
            # print(f"lat: {lat} lon: {lon}")
            return lat, lon
        else:
            # print("未找到匹配的数值")
            return None, None
    except Exception as e:
        print(f"提取失败: {str(e)}")
        return None, None
    
def extract_path_param(text):
    """使用正则表达式提取lat和lon数值"""
    try:
        # 匹配lat和lon后的数字（支持正负浮点数）
        pattern = r'local_path\.x:\s*(-?\d+\.\d+).*?local_path\.y:\s*(-?\d+\.\d+).*?local_path\.phi:\s*(-?\d+\.\d+).*?local_path\.kappa:\s*(-?\d+\.\d+)'
        match = re.search(pattern, text)
        
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            phi = float(match.group(1))
            kappa = float(match.group(2))
            # print(f"x: {x} y: {y}")
            return x, y, phi, kappa
        else:
            # print("未找到匹配的数值")
            return None, None, None, None
    except Exception as e:
        print(f"提取失败: {str(e)}")
        return None, None, None, None

def extract_vehicle_data(file_path):
    """
    从文本文件中提取车辆控制数据
    
    参数:
        file_path: 文本文件路径
        
    返回:
        包含x, y, v, phi, steer数组的字典
    """
    # 初始化存储数组
    x_values = []
    y_values = []
    v_values = []
    phi_values = []
    steer_values = []
    location_x = []
    location_y = []
    location_yaw = []
    match_point_x = []
    match_point_y = []
    match_point_v = []
    match_point_phi = []
    match_point_kappa = []
    error_d = []
    error_dot_d = []
    error_phi = []
    error_dot_phi = []
    error_s = []
    error_dot_s = []
    error_dot_dot_s = []
    commond_lat = []
    commond_lon = []
    LQR_RESULT = []
    local_path_x = []
    local_path_y = []
    local_path_phi = []
    local_path_kappa = []

    try:
        # 打开文件并逐行读取
        with open(file_path, 'r') as file:

            # for line in islice(file, 5000, 10000):  
            for line in file:
                # 检查行中是否包含目标字符串
                if "current_point,x,y,v,phi,steer:" in line:
                    # 提取目标字符串后的部分
                    data_part = line.split("current_point,x,y,v,phi,steer:")[1].strip()
                    
                    # 使用正则表达式提取所有数字
                    numbers = re.findall(r'[-+]?\d+\.\d+|[-+]?\d+', data_part)
                    
                    # 确保提取到了5个数字
                    if len(numbers) == 5:
                        # 将字符串转换为浮点数并添加到对应数组
                        x_values.append(float(numbers[0]))
                        y_values.append(float(numbers[1]))
                        v_values.append(float(numbers[2]))
                        phi_values.append(float(numbers[3]))
                        steer_values.append(float(numbers[4]))
                if "match_point,x,y,v,phi,kappa:" in line:
                    # 提取目标字符串后的部分
                    data_part = line.split("match_point,x,y,v,phi,kappa:")[1].strip()
                    
                    # 使用正则表达式提取所有数字
                    numbers = re.findall(r'[-+]?\d+\.\d+|[-+]?\d+', data_part)
                    
                    # 确保提取到了5个数字
                    if len(numbers) == 5:
                        # 将字符串转换为浮点数并添加到对应数组
                        match_point_x.append(float(numbers[0]))
                        match_point_y.append(float(numbers[1]))
                        match_point_v.append(float(numbers[2]))
                        match_point_phi.append(float(numbers[3]))
                        match_point_kappa.append(float(numbers[4]))
                if "location receive success, x: ,y: ,yaw:" in line:
                    # 提取目标字符串后的部分
                    data_part = line.split("location receive success, x: ,y: ,yaw:")[1].strip()
                    
                    # 使用正则表达式提取所有数字
                    numbers = re.findall(r'[-+]?\d+\.\d+|[-+]?\d+', data_part)
                    
                    # 确保提取到了5个数字
                    if len(numbers) == 3:
                        # 将字符串转换为浮点数并添加到对应数组
                        location_x.append(float(numbers[0]))
                        location_y.append(float(numbers[1]))
                        location_yaw.append(float(numbers[2]))
                if "lat_error:error_d,error_dot_d,error_phi,error_dot_phi:" in line:
                    # 提取目标字符串后的部分
                    data_part = line.split("lat_error:error_d,error_dot_d,error_phi,error_dot_phi:")[1].strip()
                    
                    # 使用正则表达式提取所有数字
                    numbers = re.findall(r'[-+]?\d+\.\d+|[-+]?\d+', data_part)
                    
                    # 确保提取到了5个数字
                    if len(numbers) == 4:
                        # 将字符串转换为浮点数并添加到对应数组
                        error_d.append(float(numbers[0]))
                        error_dot_d.append(float(numbers[1]))
                        error_phi.append(float(numbers[2]))
                        error_dot_phi.append(float(numbers[2]))
                if "lon_error:error_s,error_dot_s,error_dot_dot_s:" in line:
                    # 提取目标字符串后的部分
                    data_part = line.split("lon_error:error_s,error_dot_s,error_dot_dot_s:")[1].strip()
                    
                    # 使用正则表达式提取所有数字
                    numbers = re.findall(r'[-+]?\d+\.\d+|[-+]?\d+', data_part)
                    
                    # 确保提取到了5个数字
                    if len(numbers) == 3:
                        # 将字符串转换为浮点数并添加到对应数组
                        error_s.append(float(numbers[0]))
                        error_dot_s.append(float(numbers[1]))
                        error_dot_dot_s.append(float(numbers[2]))
                if "LQR RESULT:" in line:
                    # 提取目标字符串后的部分
                    data_part = line.split("LQR RESULT:")[1].strip()
                    
                    # 使用正则表达式提取所有数字
                    numbers = re.findall(r'[-+]?\d+\.\d+|[-+]?\d+', data_part)
                    
                    # 确保提取到了5个数字
                    if len(numbers) == 1:
                        # 将字符串转换为浮点数并添加到对应数组
                        LQR_RESULT.append(float(numbers[0]))
                if "commond,lat :" in line:

                    # lat,lon = extract_numbers_with_regex(line)
                    # commond_lat.append(lat)
                    # commond_lon.append(lon)
                    data_part = line.split("commond,lat :")[1].strip()
                    
                    # 使用正则表达式提取所有数字
                    numbers = re.findall(r'[-+]?\d+\.\d+|[-+]?\d+', data_part)
                    
                    if len(numbers) == 1:
                        # 将字符串转换为浮点数并添加到对应数组
                        commond_lat.append(float(numbers[0]))    
                if "commond, lon :" in line:

                    data_part = line.split("commond, lon :")[1].strip()
                    
                    # 使用正则表达式提取所有数字
                    numbers = re.findall(r'[-+]?\d+\.\d+|[-+]?\d+', data_part)
                    
                    if len(numbers) == 1:
                        # 将字符串转换为浮点数并添加到对应数组
                        commond_lon.append(float(numbers[0]))         
                if "local_path.index:" in line:
                    
                    x,y,phi,kappa = extract_path_param(line)
                    local_path_x.append(x)
                    local_path_y.append(y)
                    local_path_phi.append(phi)
                    local_path_kappa.append(kappa)
    
    except FileNotFoundError:
        print(f"错误：文件 {file_path} 不存在")
    except Exception as e:
        print(f"处理文件时发生错误: {e}")
    
    # 返回存储数据的字典
    return {
        'x': x_values,
        'y': y_values,
        'v': v_values,
        'phi': phi_values,
        'steer': steer_values,
        'location_x': location_x,
        'location_y': location_y,
        'location_yaw': location_yaw,
        'match_point_x': match_point_x,
        'match_point_y': match_point_y,
        'match_point_v': match_point_v,
        'match_point_phi': match_point_phi,
        'match_point_kappa': match_point_kappa,        
        'error_d': error_d,
        'error_dot_d': error_dot_d,
        'error_phi': error_phi,
        'error_dot_phi': error_dot_phi,
        'error_s': error_s,
        'error_dot_s': error_dot_s,
        'error_dot_dot_s': error_dot_dot_s,
        'commond_lat': commond_lat,
        'commond_lon': commond_lon,
        'LQR_RESULT': LQR_RESULT,
        'local_path_x': local_path_x,
        'local_path_y': local_path_y,
        'local_path_phi': local_path_phi,
        'local_path_kappa': local_path_kappa
               
    }

def plot_data(data, save_path=None):
    """绘制多维度数据的时间序列图，支持保存图表"""
    if not data:
        print("❌ 无数据可绘制")
        return
    
    # 创建画布和子图布局（2x3网格，使用5个子图）
    fig, axes = plt.subplots(2, 3, figsize=(12, 8), dpi=300)
    axes = axes.flatten()  # 转换为一维数组方便索引
    
    # 定义数据与图表配置的映射
    data_config = [
        ('x', 'X', 'm', 'blue'),
        ('y', 'Y', 'm', 'green'),
        ('v', 'speed', 'm/s', 'orange'),
        ('phi', 'yaw', 'rad', 'red'),
        ('steer', 'steer', 'rad', 'purple')
    ]
    
    # 绘制每个维度的数据
    for i, (key, title, unit, color) in enumerate(data_config):
        ax = axes[i]
        ax.plot(data[key], color=color, linewidth=1.5, alpha=0.8)
        ax.set_title(f'{title}', fontsize=10)
        ax.set_xlabel('t', fontsize=8)
        ax.set_ylabel(f'{title} ({unit})', fontsize=8)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.tick_params(axis='both', labelsize=6)
        
        # 优化Y轴显示（仅对数值型数据）
        if key in ['v', 'phi', 'steer']:
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.4f'))
    
    # 隐藏空的子图
    if len(data_config) < 6:
        axes[-1].axis('off')
    
    # 整体优化
    plt.suptitle('vehicle control data', fontsize=12, y=0.95)
    plt.tight_layout(rect=[0, 0, 1, 0.92])  # 预留标题空间
    
    # 保存图表（如果指定路径）
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"✅ 图表已保存至: {save_path}")    
    
def plot_comparison_chart(data, save_path=None):
    """绘制多维度数据的时间序列图，支持保存图表"""
    if not data:
        print("❌ 无数据可绘制")
        return
    
    # 创建画布和子图布局（2x3网格，使用5个子图）
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), dpi=300)
    axes = axes.flatten()  # 转换为一维数组方便索引
    
    # 定义数据与图表配置的映射
    data_config = [
        # ('error_phi', 'error_phi', 'm', 'blue'),
        # ('error_d', 'error_d', 'm', 'green'),
        ('match_point_phi', 'match_point_phi', 'm/s', 'orange'),
        ('match_point_kappa', 'match_point_kappa', 'rad', 'red'),
        ('commond_lat', 'commond_lat', 'rad', 'purple')
    ]
    
    # 绘制每个维度的数据
    for i, (key, title, unit, color) in enumerate(data_config):
        ax = axes[i]
        ax.plot(data[key], color=color, linewidth=1.5, alpha=0.8)
        ax.set_title(f'{title}', fontsize=10)
        ax.set_xlabel('t', fontsize=8)
        ax.set_ylabel(f'{title} ({unit})', fontsize=8)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.tick_params(axis='both', labelsize=6)
        
        # 优化Y轴显示（仅对数值型数据）
        if key in ['error_phi', 'error_d', 'commond_lat']:
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.4f'))
    
    # 隐藏空的子图
    if len(data_config) < 6:
        axes[-1].axis('off')
    
    # 整体优化
    plt.suptitle('vehicle control data', fontsize=12, y=0.95)
    plt.tight_layout(rect=[0, 0, 1, 0.92])  # 预留标题空间
    
    # 保存图表（如果指定路径）
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"✅ 图表已保存至: {save_path}")    


def plot_error_data(data, save_path=None):
    """绘制多维度数据的时间序列图，支持保存图表"""
    if not data:
        print("❌ 无数据可绘制")
        return
    
    # 创建画布和子图布局（2x3网格，使用5个子图）
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), dpi=300)
    axes = axes.flatten()  # 转换为一维数组方便索引
    
    # 定义数据与图表配置的映射
    data_config = [
        ('error_d', 'error_d', 'm', 'blue'),
        ('error_phi', 'error_phi', 'm', 'green'),
        ('error_s', 'error_s', 'm/s', 'orange'),        
    ]
    
    # 绘制每个维度的数据
    for i, (key, title, unit, color) in enumerate(data_config):
        ax = axes[i]
        ax.plot(data[key], color=color, linewidth=1.0, alpha=0.8)
        ax.set_title(f'{title}', fontsize=10)
        ax.set_xlabel('t', fontsize=8)
        ax.set_ylabel(f'{title} ({unit})', fontsize=8)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.tick_params(axis='both', labelsize=6)
        
        # 优化Y轴显示（仅对数值型数据）
        if key in ['error_d', 'error_phi', 'error_s']:
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.4f'))
    
    # 整体优化
    plt.suptitle('cobtrol error data', fontsize=12, y=0.95)
    plt.tight_layout(rect=[0, 0, 1, 0.92])  # 预留标题空间
    
    # 保存图表（如果指定路径）
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"✅ 图表已保存至: {save_path}")    
    
def plot_path_data(data, save_path=None):
    """绘制多维度数据的时间序列图，支持保存图表"""
    if not data:
        print("❌ 无数据可绘制")
        return
    # 创建画布
    plt.figure(figsize=(8, 6), dpi=300)

    # 绘制曲线图
    plt.plot(data['x'], data['y'], color='blue', linewidth=1, alpha=0.8, linestyle='-',label='current path')

    plt.plot(data['match_point_x'], data['match_point_y'], color='red', linewidth=1, alpha=0.8, linestyle='-', label='refer path')

    # 添加标题和坐标轴标签
    plt.title('path', fontsize=14)
    plt.xlabel('X', fontsize=12)
    plt.ylabel('Y', fontsize=12)

    # 添加图例（如果有多个曲线）
    plt.legend(fontsize=10)

    # 优化图表样式
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    plt.tight_layout()
     # 保存图表（如果指定路径）
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"✅ 图表已保存至: {save_path}")   
def plot_local_path_data(data, save_path=None):
    """绘制多维度数据的时间序列图，支持保存图表"""
    if not data:
        print("❌ 无数据可绘制")
        return
    # 创建画布
    plt.figure(figsize=(8, 6), dpi=300)

    # 绘制曲线图
    plt.plot(data['local_path_x'], data['local_path_y'], color='blue', linewidth=2, alpha=0.8, linestyle='-',label='local path')

    # 添加标题和坐标轴标签
    plt.title('local_path', fontsize=14)
    plt.xlabel('X', fontsize=12)
    plt.ylabel('Y', fontsize=12)

    # 添加图例（如果有多个曲线）
    plt.legend(fontsize=10)

    # 优化图表样式
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    plt.tight_layout()
     # 保存图表（如果指定路径）
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"✅ 图表已保存至: {save_path}")   
def plot_control_commond_data(data, save_path=None):
    """绘制多维度数据的时间序列图，支持保存图表"""
    if not data:
        print("❌ 无数据可绘制")
        return
    
    # 创建画布和子图布局（2x3网格，使用5个子图）
    fig, axes = plt.subplots(1, 3, figsize=(12, 8), dpi=300)
    axes = axes.flatten()  # 转换为一维数组方便索引
    
    # 定义数据与图表配置的映射
    data_config = [
        ('LQR_RESULT', 'LQR_RESULT', 'm', 'blue'),
        ('commond_lat', 'commond_lat', 'm', 'green'),
        ('commond_lon', 'commond_lon', 'm/s', 'orange'),        
    ]
    
    # 绘制每个维度的数据
    for i, (key, title, unit, color) in enumerate(data_config):
        ax = axes[i]
        ax.plot(data[key], color=color, linewidth=1.0, alpha=0.8)
        ax.set_title(f'{title}', fontsize=10)
        ax.set_xlabel('t', fontsize=8)
        ax.set_ylabel(f'{title} ({unit})', fontsize=8)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.tick_params(axis='both', labelsize=6)
        
        # 优化Y轴显示（仅对数值型数据）
        if key in ['error_d', 'error_phi', 'error_s']:
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.4f'))
    
    # 整体优化
    plt.suptitle('cobtrol error data', fontsize=12, y=0.95)
    plt.tight_layout(rect=[0, 0, 1, 0.92])  # 预留标题空间
    
    # 保存图表（如果指定路径）
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"✅ 图表已保存至: {save_path}") 

def plot_match_point_phi(data, save_path=None):
    """绘制多维度数据的时间序列图，支持保存图表"""
    if not data:
        print("❌ 无数据可绘制")
        return
    # 创建画布
    plt.figure(figsize=(24, 16), dpi=300)

    # 绘制曲线图
    plt.plot(data['match_point_phi'], color='blue', linewidth=2, alpha=0.8, linestyle='-',label='match_point_phi')

    # 添加标题和坐标轴标签
    plt.title('match_point_phi', fontsize=14)
    plt.xlabel('X', fontsize=12)
    plt.ylabel('Y', fontsize=12)

    # 添加图例（如果有多个曲线）
    plt.legend(fontsize=10)

    # 优化图表样式
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    plt.tight_layout()
     # 保存图表（如果指定路径）
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"✅ 图表已保存至: {save_path}") 

def plot_match_point_v(data, save_path=None):
    """绘制多维度数据的时间序列图，支持保存图表"""
    if not data:
        print("❌ 无数据可绘制")
        return
    # 创建画布
    plt.figure(figsize=(24, 16), dpi=300)

    # 绘制曲线图
    plt.plot(data['match_point_v'], color='blue', linewidth=2, alpha=0.8, linestyle='-',label='match_point_v')

    # 添加标题和坐标轴标签
    plt.title('match_point_v', fontsize=14)
    plt.xlabel('X', fontsize=12)
    plt.ylabel('Y', fontsize=12)

    # 添加图例（如果有多个曲线）
    plt.legend(fontsize=10)

    # 优化图表样式
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    plt.tight_layout()
     # 保存图表（如果指定路径）
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=300)
        print(f"✅ 图表已保存至: {save_path}") 

def main():
    # 文件路径，根据实际情况修改
    file_path = "/home/nvidia/IGV/src/control/logs/control_node.INFO"
    
    # 1. 提取数据
    print("正在提取数据...")
    data = extract_vehicle_data(file_path)
    
    if not data or len(data['x']) == 0:
        print("❌ 未提取到有效数据")
        return
    
    print(f"✅ 成功提取 {len(data['x'])} 条数据")
    print(f"✅ 成功提取 {len(data['error_d'])} 条数据")
    print(f"✅ 成功提取 {len(data['commond_lat'])} 条数据")
    print(f"✅ 成功提取 {len(data['local_path_x'])} 条数据")
    # 2. 绘制图表
    print("正在绘制可视化图表...")
    plot_data(data, save_path="/home/nvidia/IGV/src/control/plot/vehicle_control_plot.png")  # 可选：指定保存路径
    plot_error_data(data, save_path="/home/nvidia/IGV/src/control/plot/control_error_plot.png")  # 可选：指定保存路径
    plot_path_data(data, save_path="/home/nvidia/IGV/src/control/plot/path.png")
    # plot_local_path_data(data, save_path="/home/nvidia/IGV/src/control/plot/local_path.png")
    plot_control_commond_data(data, save_path="/home/nvidia/IGV/src/control/plot/control_commond.png")
    plot_comparison_chart(data, save_path="/home/nvidia/IGV/src/control/plot/control_commond.png")
    plot_match_point_phi(data, save_path="/home/nvidia/IGV/src/control/plot/match_point_phi.png")
    plot_match_point_v(data, save_path="/home/nvidia/IGV/src/control/plot/match_point_v.png")
    plt.show()

if __name__ == "__main__":
    plt.close('all') 
    main()

