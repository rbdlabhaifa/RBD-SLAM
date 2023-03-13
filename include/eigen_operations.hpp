#ifndef EIGEN_OPERATIONS_H_
#define EIGEN_OPERATIONS_H_

#include <Eigen/Core>

namespace EigenOperations {

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_indices(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        Eigen::Matrix<T, Eigen::Dynamic, 1> block_mat(indices.size());

        for (Eigen::Index i = 0; i < indices.rows(); ++i) {
            block_mat(i) = vec(indices[i]);
        }

        return block_mat;
    }

    template <typename T>
    void remove_row(Eigen::Matrix<T, Eigen::Dynamic, 1>& matrix,
                    Eigen::Index rowToRemove) {
        Eigen::Index numRows = matrix.rows() - 1;
        Eigen::Index numCols = matrix.cols();

        if (rowToRemove < numRows) {
            matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) =
                matrix.block(rowToRemove + 1, 0, numRows - rowToRemove,
                             numCols);
        }

        matrix.conservativeResize(numRows, numCols);
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_all_indices_except(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        auto block_vec = vec;

        for (Eigen::Index i = 0; i < indices.size(); ++i) {
            remove_row(block_vec, indices[i]);
        }

        return block_vec;
    }

    template <typename T>
    std::vector<T> get_indices(
        const std::vector<T>& vec,
        const Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1>& indices) {
        std::vector<T> v_block(indices.size());

        for (Eigen::Index i = 0; i < indices.rows(); ++i) {
            v_block[i] = vec[indices[i]];
        }

        return v_block;
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_indices(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec,
        const std::vector<std::size_t>& indices) {
        Eigen::Matrix<T, Eigen::Dynamic, 1> block_mat(indices.size());

        for (Eigen::Index i = 0; i < indices.size(); ++i) {
            block_mat(i) = vec(indices[i]);
        }

        return block_mat;
    }

    std::vector<float> sum_rows(const Eigen::VectorXf& mat);

    void sort_matrix_columns(Eigen::MatrixXf& mat);

    Eigen::VectorXf gradient(const Eigen::VectorXf& v);
}  // namespace EigenOperations

#endif  // EIGEN_OPERATIONS_H_
