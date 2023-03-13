#include "eigen_operations.hpp"

namespace EigenOperations {
    std::vector<float> sum_rows(const Eigen::VectorXf& mat) {
        std::vector<float> sum_vec(mat.cols());

        for (Eigen::Index i = 0; i < mat.cols(); ++i) {
            sum_vec[i] = mat.col(i).sum();
        }

        return sum_vec;
    }

    void sort_matrix_columns(Eigen::MatrixXf& mat) {
        for (Eigen::Index col = 0; col < mat.cols(); ++col) {
            std::vector<float> col_vec(mat.rows());

            for (Eigen::Index row = 0; row < mat.rows(); ++row) {
                col_vec[row] = mat(row, col);
            }

            std::sort(col_vec.begin(), col_vec.end());

            for (Eigen::Index row = 0; row < mat.rows(); ++row) {
                mat(row, col) = col_vec[row];
            }
        }
    }

    Eigen::VectorXf gradient(const Eigen::VectorXf& v) {
        if (v.size() <= 1) return v;

        Eigen::VectorXf res(v.size());
        for (Eigen::Index j = 0; j < v.size(); j++) {
            Eigen::Index j_left = j - 1;
            Eigen::Index j_right = j + 1;
            if (j_left < 0) {
                j_left = 0;
                j_right = 1;
            }
            if (j_right >= v.size()) {
                j_right = v.size() - 1;
                j_left = j_right - 1;
            }
            // gradient value at position j
            float dist_grad = (v(j_right) - v(j_left)) / 2.0;
            res(j) = dist_grad;
        }
        return res;
    }

}  // namespace EigenOperations
