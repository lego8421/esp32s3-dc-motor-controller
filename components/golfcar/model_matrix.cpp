#include "model_matrix.h"

#include <cmath>

// ModelMatrix
ModelMatrix::ModelMatrix()
    : row_(4), column_(4) {
    element_.resize(row_ * column_);
}

ModelMatrix::ModelMatrix(const ModelMatrix &other)
    : row_(other.row()), column_(other.column()) {
    element_ = other.element();
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column)
    : row_(row), column_(column) {
    element_.resize(row_ * column_);
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const double *element)
    : row_(row), column_(column), element_(element, element + (row * column)) {
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const double **element)
    : row_(row), column_(column) {
    element_.resize(row_ * column_);

    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            element_[r * column + c] = element[r][c];
        }
    }
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const std::vector<double> element)
    : row_(row), column_(column), element_(element) {
}

ModelMatrix::~ModelMatrix() {
    element_.clear();
}

unsigned int ModelMatrix::row() const {
    return row_;
}

unsigned int ModelMatrix::column() const {
    return column_;
}

std::vector<double> ModelMatrix::element() const {
    return element_;
}

double ModelMatrix::get(const unsigned int row, const unsigned int column) const {
    if (row > row_) {
        return 0.0;
    } else if (column > column_) {
        return 0.0;
    } else {
        return element_[row * column_ + column];
    }
}

void ModelMatrix::set(const unsigned int row, const unsigned int column, const double value) {
    if (row > row_) {
        return;
    } else if (column > column_) {
        return;
    } else {
        element_[row * column_ + column] = value;
    }
}

ModelMatrix ModelMatrix::zero(const unsigned int row, const unsigned int column) {
    return ModelMatrix(row, column);
}

ModelMatrix ModelMatrix::one(const unsigned int row, const unsigned int column) {
    std::vector<double> mat(row * column);
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            mat[r * column + c] = 1.0;
        }
    }
    return ModelMatrix(row, column, mat);
}

ModelMatrix ModelMatrix::identity(const unsigned int row, const unsigned int column) {
    std::vector<double> mat(row * column);
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            if (r == c) {
                mat[r * column + c] = 1.0;
            } else {
                mat[r * column + c] = 0.0;
            }
        }
    }
    return ModelMatrix(row, column, mat);
}

ModelMatrix ModelMatrix::transpose() {
    std::vector<double> ele(row_ * column_);
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            ele[c * row_ + r] = element_[r * column_ + c];
        }
    }
    return ModelMatrix(column_, row_, ele);
}

double ModelMatrix::determinant() {
    if (row_ == column_) {
        return determinant(element_, row_);
    } else if (row_ > column_) {
        return determinant((this->transpose() * (*this)).element(), column_);
    } else {
        return determinant(((*this) * this->transpose()).element(), row_);
    }
}

ModelMatrix ModelMatrix::inverse() {
    if (row_ == column_) {
        // square matrix
        return ModelMatrix(row_, column_,  matrixInversion(element_, row_));
    } else {
        // rectangular matrix
        return pseudoInverse();
    }
}

ModelMatrix ModelMatrix::inverse(const double sigma) {
    if (row_ <= column_) {
        // m by n matrix (n >= m)
        // generate sigma digonal matrix
        ModelMatrix temp = ModelMatrix::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        return this->transpose() * ((*this) * (this->transpose()) + temp).inverse();
    } else {
        // generate sigma digonal matrix
        ModelMatrix temp = ModelMatrix::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        return (this->transpose() * (*this) + temp).inverse() * this->transpose();
    }
}

double ModelMatrix::length() const {
    double l = 0.0;
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            l += element_[r * column_ + c] * element_[r * column_ + c];
        }
    }
    return std::sqrt(l);
}

ModelMatrix ModelMatrix::normalize() const {
    double l = length();
    if (l == 0.0) {
        return ModelMatrix::identity(row_, column_);
    } else {
        std::vector<double> ele(row_, column_);
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                ele[r * column_ + c] = element_[r * column_ + c] / l;
            }
        }
        return ModelMatrix(row_, column_, ele);
    }
}

double ModelMatrix::dot(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        double dot = 0.0;
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                dot += element_[r * column_ + c] * rhs.element()[r * column_ + c];
            }
        }
        return dot;
    } else {
        return 0.0;
    }
}

ModelMatrix ModelMatrix::cross(const ModelMatrix &rhs) {
    if (row_ == 3 && column_ == 1 && rhs.row() == 3 && rhs.column() == 1) {
        std::vector<double> ele(3);
        ele[0] = element_[1] * rhs.element()[2] - element_[2] * rhs.element()[1];
        ele[1] = element_[2] * rhs.element()[0] - element_[0] * rhs.element()[2];
        ele[2] = element_[0] * rhs.element()[1] - element_[1] * rhs.element()[0];
        return ModelMatrix(row_, column_, ele);
    } else {
        return ModelMatrix::zero(3, 1);
    }
}

ModelMatrix ModelMatrix::cross() {
    if (row_ == 3 && column_ == 1) {
        std::vector<double> ele(3 * 3);
        ele[0 * 3 + 0] = 0.0;
        ele[0 * 3 + 1] = -element_[2];
        ele[0 * 3 + 2] = element_[1];
        ele[1 * 3 + 0] = element_[2];
        ele[1 * 3 + 1] = 0.0;
        ele[1 * 3 + 2] = -element_[0];
        ele[2 * 3 + 0] = -element_[1];
        ele[2 * 3 + 1] = element_[0];
        ele[2 * 3 + 2] = 0.0;
        return ModelMatrix(3, 3, ele);
    } else {
        return ModelMatrix::zero(3, 3);
    }
}

ModelMatrix &ModelMatrix::operator=(const ModelMatrix &other) {
    this->row_ = other.row_;
    this->column_ = other.column();
    this->element_ = other.element();
    return *this;
}

ModelMatrix ModelMatrix::operator+(const double &rhs) {
    ModelMatrix right = ModelMatrix::one(row_, column_) * rhs;
	return (*this) + right;
}

ModelMatrix ModelMatrix::operator+(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        std::vector<double> temp(row_ * column_);
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                temp[r * column_ + c] = element_[r * column_ + c] + rhs.element()[r * column_ + c];
            }
        }
        return ModelMatrix(row_, column_, temp);
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix ModelMatrix::operator-(const double &rhs) {
    ModelMatrix right = ModelMatrix::one(row_, column_) * rhs;
	return (*this) - right;
}

ModelMatrix ModelMatrix::operator-(const ModelMatrix &rhs) {
    if (row_ == rhs.row() && column_ == rhs.column()) {
        std::vector<double> temp(row_ * column_);
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                temp[r * column_ + c] = element_[r * column_ + c] - rhs.element()[r * column_ + c];
            }
        }
        return ModelMatrix(row_, column_, temp);
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix ModelMatrix::operator*(const double &rhs) {
    std::vector<double> temp(row_ * column_);
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            temp[r * column_ + c] = element_[r * column_ + c] * rhs;
        }
    }
    return ModelMatrix(row_, column_, temp);
}

ModelMatrix ModelMatrix::operator*(const ModelMatrix &rhs) {
    if (column_ == rhs.row()) {
		std::vector<double> temp(row_ * rhs.column());
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < rhs.column(); c++) {
                temp[r * rhs.column() + c] = 0;
                for (unsigned int k = 0; k < column_; k++) {
                    temp[r * rhs.column() + c] += element_[r * column_ + k] * rhs.element()[k * rhs.column() + c];
                }
            }
        }
        return ModelMatrix(row_, rhs.column(), temp);
    } else {
		return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix operator+(const double &lhs, const ModelMatrix &rhs) {
    ModelMatrix left = ModelMatrix::one(rhs.row(), rhs.column()) * lhs;
    return left + rhs;
}

ModelMatrix operator-(const double &lhs, const ModelMatrix &rhs) {
    ModelMatrix left = ModelMatrix::one(rhs.row(), rhs.column()) * lhs;
    return left - rhs;
}

ModelMatrix operator*(const double &lhs, const ModelMatrix &rhs) {
    std::vector<double> temp(rhs.row() * rhs.column());
    for (unsigned int r = 0; r < rhs.row(); r++) {
        for (unsigned int c = 0; c < rhs.column(); c++) {
            temp[r * rhs.column() + c] = rhs.element()[r * rhs.column() + c] * lhs;
        }
    }
    return ModelMatrix(rhs.row(), rhs.column(), temp);
}

ModelMatrix ModelMatrix::pseudoInverse() {
    if (row_ == column_) {
        return inverse();
    } else if (row_ > column_) {
        return pseudoInverseL();
    } else {
        return pseudoInverseR();
    }
}

ModelMatrix ModelMatrix::pseudoInverseR() {
    return this->transpose() * ((*this) * (this->transpose())).inverse();
}

ModelMatrix ModelMatrix::pseudoInverseL() {
    return ((this->transpose()) * (*this)).inverse() * this->transpose();
}

double ModelMatrix::determinant(std::vector<double> matrix, int order) {
    // the determinant value
    double det = 1.0;

    // stop the recursion when matrix is a single element
    if (order == 1) {
        det = matrix[0];
    } else if (order == 2) {
        det = matrix[0 * 2 + 0] * matrix[1 * 2 + 1] - matrix[0 * 2 + 1] * matrix[1 * 2 + 0];
    } else if (order == 3) {
        det = matrix[0 * 3 + 0] * matrix[1 * 3 + 1] * matrix[2 * 3 + 2] + matrix[0 * 3 + 1] * matrix[1 * 3 + 2] * matrix[2 * 3 + 0] + matrix[0 * 3 + 2] * matrix[1 * 3 + 0] * matrix[2 * 3 + 1] - matrix[0 * 3 + 0] * matrix[1 * 3 + 2] * matrix[2 * 3 + 1] - matrix[0 * 3 + 1] * matrix[1 * 3 + 0] * matrix[2 * 3 + 2] - matrix[0 * 3 + 2] * matrix[1 * 3 + 1] * matrix[2 * 3 + 0];
    } else {
        // generation of temporary matrix
        std::vector<double> temp_matrix = matrix;

        // gaussian elimination
        for (int i = 0; i < order; i++) {
            // find max low
            double temp = 0.000;
            int max_row = i;
            for (int j = i; j < order; j++) {
                if (std::abs(temp_matrix[j * order + i]) > temp) {
                    temp = std::abs(temp_matrix[j * order + i]);
                    max_row = j;
                }
            }
            if (std::abs(temp_matrix[max_row * order + i]) > 0.0001) {
                // transfer row
                if (max_row != i) {
                    for (int j = 0; j < order; j++) {
                        temp = -temp_matrix[max_row * order + j];
                        temp_matrix[max_row * order + j] = temp_matrix[i * order + j];
                        temp_matrix[i * order + j] = temp;
                    }
                }
                // elemination
                for (int j = i + 1; j < order; j++) {
                    temp = temp_matrix[j * order + i] / temp_matrix[i * order + i];
                    for (int k = i; k < order; k++) {
                        temp_matrix[j * order + k] -= temp_matrix[i * order + k] * temp;
                    }
                }
            }
        }

        for (int i = 0; i < order; i++) {
            det *= temp_matrix[i * order + i];
        }
    }

    return det;
}

std::vector<double> ModelMatrix::matrixInversion(std::vector<double> matrix, int order) {
    std::vector<double> matA = matrix;
    std::vector<double> matB = ModelMatrix::identity(order, order).element();

    // Gauss-Jordan
    // Forward
    for (int i = 0; i < order; i++) {
        // max row
        double temp = 0.000;
        int max_row = i;
        for (int j = i; j < order; j++) {
            if (std::abs(matA[j * order + i]) > temp) {
                temp = std::abs(matA[j * order + i]);
                max_row = j;
            }
        }
        // change row
        double temp2 =  matA[max_row * order + i];
        for (int j = 0; j < order; j++) {
            temp = matA[max_row * order + j];
            matA[max_row * order + j] = matA[i * order + j];
            matA[i * order + j] = temp / temp2;

            temp = matB[max_row * order + j];
            matB[max_row * order + j] = matB[i * order + j];
            matB[i * order + j] = temp / temp2;
        }
        for (int j = i + 1; j < order; j++) {
            temp = matA[j * order + i];
            for (int k = 0; k < order; k++) {
                matA[j * order + k] -= matA[i * order + k] * temp;
                matB[j * order + k] -= matB[i * order + k] * temp;
            }
        }
    }

    //Backward
    for (int i = order - 1; i >= 0; i--) {
        for (int j = i - 1; j >= 0; j--) {
            double temp = matA[j * order + i];
            for (int k = 0; k < order; k++) {
                matA[j * order + k] -= matA[i * order + k] * temp;
                matB[j * order + k] -= matB[i * order + k] * temp;
            }
        }
    }

    return matB;
}
