#include "eigen_operations.hpp"

namespace EigenOperations
{
std::vector<float> sum_rows(const Eigen::VectorXf &mat)
{
    std::vector<float> sum_vec(mat.cols());

    for (Eigen::Index i = 0; i < mat.cols(); ++i)
    {
        sum_vec[i] = mat.col(i).sum();
    }

    return sum_vec;
}

void sort_matrix_columns(Eigen::MatrixXf &mat)
{
    for (Eigen::Index col = 0; col < mat.cols(); ++col)
    {
        std::vector<float> col_vec(mat.rows());

        for (Eigen::Index row = 0; row < mat.rows(); ++row)
        {
            col_vec[row] = mat(row, col);
        }

        std::sort(col_vec.begin(), col_vec.end());

        for (Eigen::Index row = 0; row < mat.rows(); ++row)
        {
            mat(row, col) = col_vec[row];
        }
    }
}

Eigen::VectorXf gradient(const Eigen::VectorXf &v)
{
    if (v.size() <= 1)
        return v;

    Eigen::VectorXf res(v.size());
    for (Eigen::Index j = 0; j < v.size(); j++)
    {
        Eigen::Index j_left = j - 1;
        Eigen::Index j_right = j + 1;
        if (j_left < 0)
        {
            j_left = 0;
            j_right = 1;
        }
        if (j_right >= v.size())
        {
            j_right = v.size() - 1;
            j_left = j_right - 1;
        }
        // gradient value at position j
        float dist_grad = (v(j_right) - v(j_left)) / 2.0;
        res(j) = dist_grad;
    }
    return res;
}

Eigen::MatrixXd vec_vec2eigen_mat(std::vector<std::vector<double>> origin_mat)
{
    // Convert the std::vector origin_mat to an Eigen origin_mat
    Eigen::MatrixXd eigenMatrix(origin_mat.size(), origin_mat[0].size());
    for (int i = 0; i < origin_mat.size(); ++i)
    {
        for (int j = 0; j < origin_mat[i].size(); ++j)
        {
            eigenMatrix(i, j) = origin_mat[i][j];
        }
    }
    return eigenMatrix;
}

std::vector<std::vector<double>> eigen_mat2vec_vec(Eigen::MatrixXd orig_mat)
{
    int rows = orig_mat.rows();
    int cols = orig_mat.cols();
    std::vector<std::vector<double>> ret_vector(rows,
                                                std::vector<double>(cols));
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            ret_vector[i][j] = orig_mat(i, j);
        }
    }
    return ret_vector;
}

std::vector<std::vector<double>>
vec_eigen3d2vec_vec(std::vector<Eigen::Vector3d> orig_vec)
{
    int rows = orig_vec.size();
    int cols = 3;
    std::vector<std::vector<double>> ret_vector;

    for (int i = 0; i < rows; ++i)
    {
        ret_vector[i] = std::vector<double>(3) = {
            orig_vec[i].x(), orig_vec[i].y(), orig_vec[i].z()};
    }
    return ret_vector;
}

std::vector<std::vector<double>>
vec_eigen_mat2vec_vec(std::vector<Eigen::Matrix<double, 3, 1>> orig_vec)
{
    std::vector<std::vector<double>> ret_vector;
    int rows = orig_vec.size();

    for (int i = 0; i < rows; ++i)
    {
        Eigen::Matrix<double, 3, 1> line = orig_vec[i];
        ret_vector.push_back(std::vector<double>{line(0), line(1), line(2)});
    }
    return ret_vector;
}

} // namespace EigenOperations
