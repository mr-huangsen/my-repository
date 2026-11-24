#include "common/qp_solve.h"
#include <fmt/ostream.h>

QpSolve::QpSolve(){}
QpSolve::~QpSolve(){}

bool QpSolve::run(Eigen::MatrixXd P,Eigen::VectorXd q,Eigen::MatrixXd A,Eigen::VectorXd l_inf,Eigen::VectorXd u_sup)
{

    // 2. 转换为 OSQP 所需的数据格式
    // 设置问题维度
    int n = P.rows();  // 变量的个数
    int m = A.rows();  // 约束的个数,
    Eigen::MatrixXd P_upperTriangular = P.triangularView<Eigen::Upper>();
    // 将 Eigen 数据转换为 OSQP 格式
    // 将P、q、A等转换为csc格式矩阵
    std::vector<c_int> rowind_P,rowind_A;
    std::vector<c_int> colptr_P,colptr_A;
    std::vector<c_float> values_P,values_A;
    createCSC(P_upperTriangular,rowind_P,colptr_P,values_P);
    createCSC(A,rowind_A,colptr_A,values_A);

    csc *P_csc = csc_matrix(n, n, values_P.size(), values_P.data(), rowind_P.data(), colptr_P.data());  // 将P转换为csc格式
    c_float* q_x = q.data();
    csc *A_csc = csc_matrix(m, n, values_A.size(), values_A.data(), rowind_A.data(), colptr_A.data());  // 将A转换为csc格式
    c_float* l_x = l_inf.data();
    c_float* u_x = u_sup.data();
    // print_csc_matrix(P_csc);   
    // print_csc_matrix(A_csc);
    // 3. 设置 OSQP 数据结构
    OSQPData* data = (OSQPData*)malloc(sizeof(OSQPData));
    data->n = n;
    data->m = m;
    data->P = P_csc;
    data->q = q_x;
    data->A = A_csc;
    data->l = l_x;
    data->u = u_x;

    // 4. 设置求解器设置
    OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
    osqp_set_default_settings(settings);
    // settings->max_iter = max_iter;  
    settings->max_iter = 1000; 
    settings->verbose = false; // 输出求解器信息
    

    // 5. 创建求解器并求解
    OSQPWorkspace* osqp_work = nullptr;
    osqp_setup(&osqp_work, data, settings);
    osqp_solve(osqp_work);

    auto status = osqp_work->info->status_val;

    if (status < 0 || (status != 1 && status != 2)) {
        LOG_INFO("P: \n{}", fmt::streamed(P.block(0,0,20,20)));
        LOG_INFO("A: \n{}", fmt::streamed(A.block(0,0,20,20)));
        LOG_INFO("q: \n{}", fmt::streamed(q.segment(0,20)));
        LOG_INFO("l_inf: \n{}", fmt::streamed(l_inf.segment(0,20)));
        LOG_INFO("u_sup: \n{}", fmt::streamed(u_sup.segment(0,20)));
        LOG_WARN("failed optimization status: {}", osqp_work->info->status);
        osqp_cleanup(osqp_work);
        if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
        }
        if (settings)  c_free(settings);  
        return false;
    } else if (osqp_work->solution == nullptr) {
        LOG_WARN("The solution from OSQP is nullptr");
        osqp_cleanup(osqp_work);
        if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
        }
        if (settings)  c_free(settings);  
    }
    // 6. 获取解并输出
    Eigen::VectorXd solution(n);
    for (int i = 0; i < n; ++i) {
        solution(i) = osqp_work->solution->x[i];
    }
    osqp_solution = solution;
    // 7. 清理资源
    osqp_cleanup(osqp_work);
    if (data) {
    if (data->A) c_free(data->A);
    if (data->P) c_free(data->P);
    c_free(data);
    }
    if (settings)  c_free(settings);    
    
    return true;
}
void QpSolve::createCSC(const Eigen::MatrixXd& P, std::vector<c_int>& rowind, std::vector<c_int>& colptr, std::vector<c_float>& values) {
    int rows = P.rows();
    int cols = P.cols();
    
    // 行索引、列指针和数值数组
    rowind.clear();
    colptr.clear();
    values.clear();
    
    // 初始化列指针
    colptr.push_back(0);  // 第一个列指针为0
    
    // 遍历矩阵的每一列，查找非零元素
    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            if (P(i, j) > 0.00001) {
                rowind.push_back(i);      // 保存行索引
                values.push_back(static_cast<float>(P(i, j))); // 保存值                
            }
        }
        colptr.push_back(rowind.size()); // 每一列的结束位置
    }
}
void QpSolve::print_csc_matrix(csc* matrix) {
    // 获取矩阵的维度和数据
    int m = matrix->m;  // 行数
    int n = matrix->n;  // 列数
    int nzmax = matrix->nzmax;  // 非零元素数量
    c_int* p = matrix->p;  // 列指针
    c_int* i = matrix->i;  // 行索引
    double* x = matrix->x;  // 非零元素值

    
    // 打印矩阵的维度
    std::cout << "Matrix dimensions: " << m << " x " << n << std::endl;
    std::cout << "Number of non-zero elements: " << nzmax << std::endl;

    //打印每个非零元素的位置和值
    for (int col = 0; col < n; ++col) {
        std::cout << "Column " << col << ": ";
        for (int idx = p[col]; idx < p[col + 1]; ++idx) {
            std::cout << "Row " << i[idx] << " = " << x[idx] << "; ";
        }
        std::cout << std::endl;
    }
}