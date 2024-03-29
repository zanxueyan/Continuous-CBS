/*
This file is part of C++lex, a project by Tommaso Urli.
C++lex is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
C++lex is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with C++lex.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pilal.h"
#include <utility>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>

using std::pair;
using std::make_pair;
using std::swap;

namespace pilal {

	/*
	Matrix
	*/

	Matrix::Matrix() :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(0)),
		rows(0),
		columns(0) {

	}

	Matrix::Matrix(char const * values) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false) {

		int chunks = 0;
		long double ignore;
		std::stringstream buffer(values);
		while (!buffer.eof()) {
			buffer >> ignore;
			++chunks;
		}

		this->rows = 1;
		this->columns = chunks;
		this->values = new storage(chunks);
		set_values(values);

	}

	Matrix::Matrix(int n) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(n * n)),
		rows(n),
		columns(n) {
	}

	Matrix::Matrix(int n, long double v) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(n * n, v)),
		rows(n),
		columns(n) {
	}

	Matrix::Matrix(int r, int c) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(r * c)),
		rows(r),
		columns(c) {
	}

	Matrix::Matrix(int r, int c, long double v) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(r * c, v)),
		rows(r),
		columns(c) {
	}

	Matrix::Matrix(Matrix const& m) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(*m.values)),
		rows(m.rows),
		columns(m.columns) {

	}

	Matrix::Matrix(AnonymousMatrix m) :
		lu_up_to_date(false),
		determinant_up_to_date(false),
		inverse_up_to_date(false),
		values(new storage(0)),
		rows(m.rows),
		columns(m.columns) {

		swap(values->contents, m.values->contents);
	}


	Matrix::~Matrix() {
		delete values;
	}


	// Operators overloading with storage accessor
	long double& Matrix::operator() (int r, int c) {
		return values->at(r * columns + c);
	}

	long double const& Matrix::operator() (int r, int c) const {
		return values->at(r * columns + c);
	}

	// Operators overloading with storage accessor
	long double& Matrix::operator() (int xloc) {
		if (rows == 1 || columns == 1)
			return values->at(xloc);
		else
			throw(NotAVectorException());
	}

	long double const& Matrix::operator() (int xloc) const {
		if (rows == 1 || columns == 1)
			return values->at(xloc);
		else
			throw(NotAVectorException());
	}

	void Matrix::set_row(int xloc, char const* row) {

		std::stringstream buffer(row);

		for (int yloc = 0; yloc < columns; ++yloc)
			buffer >> at(xloc, yloc);
	}

	void Matrix::set_column(int yloc, char const* column) {

		std::stringstream buffer(column);

		for (int xloc = 0; xloc < rows; ++xloc)
			buffer >> at(xloc, yloc);

	}

	void Matrix::set_values(char const* values) {

		std::stringstream buffer(values);

		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				buffer >> at(xloc, yloc);

	}

	// Matrix::storage_accessor
	long double& Matrix::at(int r, int c) {
		if (r >= rows || c >= columns)
			throw(IndexOutOfBoundException());

		//long double& dest = values->at(r * columns + c);
		//return Matrix::storage_accessor(dest, *this);
		return values->at(r * columns + c);
	}

	long double const& Matrix::at(int r, int c) const {
		if (r >= rows || c >= columns)
			throw(IndexOutOfBoundException());

		//long double& dest = values->at(r * columns + c);
		return values->at(r * columns + c);
	}

	AnonymousMatrix Matrix::operator*(Matrix const& m) {
		if (columns != m.rows)
			throw(SizeMismatchException());

		// Allocate return matrix filled with zeroes
		AnonymousMatrix r(rows, m.columns);

		// Core computation
		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < m.columns; ++yloc)
				for (int h = 0; h < columns; ++h)
					r(xloc, yloc) += at(xloc, h) * m(h, yloc);
		return r;
	}

	Matrix& Matrix::operator*=(Matrix const& m) {

		// Multiplication can be carried out
		if (columns != m.rows)
			throw(SizeMismatchException());

		// No fear to change matrix size
		Matrix r(rows, m.columns);

		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < m.columns; ++yloc)
				for (int h = 0; h < columns; ++h)
					r(xloc, yloc) += at(xloc, h) * m(h, yloc);

		// Swap contents
		swap(values->contents, r.values->contents);
		rows = r.rows;
		columns = r.columns;

		return *this;
	}


	Matrix& Matrix::operator*=(AnonymousMatrix m) {

		// Multiplication can be carried out
		if (columns != m.rows)
			throw(SizeMismatchException());

		// No fear to change matrix size
		Matrix r(rows, m.columns);

		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < m.columns; ++yloc)
				for (int h = 0; h < columns; ++h)
					r(xloc, yloc) += at(xloc, h) * m(h, yloc);

		// Swap contents
		swap(values->contents, r.values->contents);
		rows = r.rows;
		columns = r.columns;

		return *this;
	}

	AnonymousMatrix Matrix::operator*(AnonymousMatrix m) const {

		if (columns != m.rows)
			throw(SizeMismatchException());

		// Allocate return matrix filled with zeroes
		AnonymousMatrix r(rows, m.columns);

		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < m.columns; ++yloc)
				for (int h = 0; h < columns; ++h)
					r(xloc, yloc) += at(xloc, h) * m(h, yloc);

		// Swap pointers
		swap(m.values->contents, r.values->contents);                         // Keep m's counter
		m.rows = r.rows;
		m.columns = r.columns;

		return m;
	}


	Matrix& Matrix::operator=(char const * values) {

		if (rows == 0 && columns == 0) {
			int chunks = 0;
			long double ignore;
			std::stringstream buffer(values);
			while (!buffer.eof()) {
				buffer >> ignore;
				++chunks;
			}
			resize(1, chunks);
		}
		set_values(values);
		return *this;
	}

	Matrix& Matrix::operator=(Matrix const& m) {

		// Handle autoassignment
		if (&m == this)
			return *this;

		// Scalars
		rows = m.rows;
		columns = m.columns;

		// Storage
		delete values;                                                          // Free old values
		values = new storage(*m.values);                                        // Deep copy of values

		return *this;
	}

	Matrix& Matrix::operator=(AnonymousMatrix m) {

		// Swap values, m will destroy old Matrix values
		swap(values->contents, m.values->contents);                          // Does not swap counters

		// Handle scalar values
		rows = m.rows;
		columns = m.columns;

		return *this;
	}

	AnonymousMatrix Matrix::operator-(AnonymousMatrix  m) const {

		if (dim() != m.dim())
			throw(SizeMismatchException());

		AnonymousMatrix r(rows, columns);

		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				r(xloc, yloc) = at(xloc, yloc) - m(xloc, yloc);

		return r;
	}

	AnonymousMatrix Matrix::operator+(AnonymousMatrix  m) const {

		if (dim() != m.dim())
			throw(SizeMismatchException());

		AnonymousMatrix r(rows, columns);

		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				r(xloc, yloc) = at(xloc, yloc) + m(xloc, yloc);

		return r;
	}

	// Utility functions
	pair<int, int> Matrix::dim() const {
		return make_pair(rows, columns);
	}

	double Matrix::space() const {
		return (rows * columns * sizeof(long double) * 0.000000954);
	}

	// Aux
	bool Matrix::more_equal_than(long double value, long double tol = 0.0000000000000001) const {
		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				if (at(xloc, yloc) + tol < value) return false;
		return true;
	}

	bool Matrix::less_equal_than(long double value, long double tol = 0.0000000000000001) const {

		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				if (at(xloc, yloc) - tol > value) return false;
		return true;
	}

	Matrix::operator long double() {
		if (dim() != make_pair(1, 1))
			throw (SizeMismatchException());
		return values->at(0);
	}

	void Matrix::log(std::string name) const {
		// Printing
		printf("-- %s\n", name.c_str());
		for (int xloc = 0; xloc < rows; ++xloc) {
			printf(" ");
			for (int yloc = 0; yloc < columns; ++yloc) {
				printf("%10.5f ", (double)at(xloc, yloc));
			}
			printf("\n");
		}
		printf("--\n");
	}

	void Matrix::logtave(std::string varname) const {
		// Printing
		printf("%s = ...\n[", varname.c_str());
		for (int xloc = 0; xloc < rows; ++xloc) {

			for (int yloc = 0; yloc < columns; ++yloc) {

				printf("%.16f ", (double)at(xloc, yloc));
			}

			printf(";\n");
		}
		printf("]\n");
	}

	bool Matrix::is_square() const {
		return (rows == columns);
	}

	void Matrix::swap_columns(int r, int w) {

		for (int xloc = 0; xloc < rows; ++xloc)
			swap(values->contents->at(xloc * columns + r), values->contents->at(xloc * columns + w));

		// Waiting for a smarter implementation
		determinant_up_to_date = false;
		lu_up_to_date = false;
		inverse_up_to_date = false;
	}

	void Matrix::swap_rows(int r, int w) {

		for (int xloc = 0; xloc < columns; ++xloc)
			swap(values->contents->at(r * columns + xloc), values->contents->at(w * columns + xloc));

		// Waiting for a smarter implementation
		determinant_up_to_date = false;
		lu_up_to_date = false;
		inverse_up_to_date = false;
	}


	bool Matrix::is_identity(long double tol) const {

		// Identity check
		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				if (((xloc == yloc) && !tol_equal(at(xloc, yloc), 1, tol)) ||
					((xloc != yloc) && !tol_equal(at(xloc, yloc), 0, tol)))
					return false;
		return true;
	}

	void Matrix::set_identity() {

		if (!is_square())
			throw(MatrixNotSquareException());

		// Set identity
		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				if (xloc == yloc)
					at(xloc, yloc) = 1;
				else
					at(xloc, yloc) = 0;

		det = 1;
		determinant_up_to_date = true;
	}

	void Matrix::set_determinant(long double d) {

		det = d;
		determinant_up_to_date = true;
	}

	long double Matrix::determinant() const {

		// Return determinant if cached, else factorize and return it
		if (!determinant_up_to_date) {
			Matrix l, u, p;
			get_lupp(l, u, p, PF_VECTOR);
		}
		return det;
	}

	void Matrix::transpose() {

		// If matrix is square just swap the elements
		if (rows == columns) {
			for (int yloc = 1; yloc < columns; ++yloc)
				for (int xloc = 0; xloc < yloc; ++xloc)
					swap(at(xloc, yloc), at(yloc, xloc));
			return;
		}

		AnonymousMatrix tps(columns, rows);
		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				tps(yloc, xloc) = at(xloc, yloc);

		*this = tps;
	}

	void Matrix::empty() {
		delete values;
		values = new storage(rows * columns);
	}

	void Matrix::resize(int r, int c) {

		// Reinitialize values
		delete values;
		values = new storage(r * c);

		// Resize
		rows = r;
		columns = c;

		// Invalid determinant
		determinant_up_to_date = false;
	}

	AnonymousMatrix Matrix::gaussian_elimination() {

		AnonymousMatrix r(*this);

		int yloc = 0, pivot = 0;
		while (yloc < columns && pivot < rows) {

			// Reset tem and tpm elements
			Matrix tem(rows, 1, 0);
			tem(pivot, 0) = 1;

			int column_max_position = pivot;
			long double max = r(column_max_position, yloc);

			// Partial pivoting process
			for (int xloc = yloc; xloc < rows; ++xloc)
				if (fabs(r(xloc, yloc)) > fabs(max)) {
					column_max_position = xloc;
					max = r(xloc, yloc);
				}

			if (max != 0) {

				// Update U and P with TPM only if necessary
				if (yloc != column_max_position)
					r.swap_rows(pivot, column_max_position);

				// Write tem vector for current column
				for (int xloc = pivot + 1; xloc < rows; ++xloc)
					tem(xloc, 0) = -(r(xloc, yloc) / max);

				// Optimization of tem * u that takes into account the
				// shape of tem and u
				for (int xloc = pivot + 1; xloc < rows; ++xloc) {
					Matrix t(1, columns, 0);

					for (int o = pivot; o < columns; ++o)
						t(o) = tem(xloc, 0) * r(yloc, o) + r(xloc, o);


					for (int k = pivot; k < columns; ++k)
						r(xloc, k) = t(k);

				}
				++pivot;
			}
			++yloc;
		}

		return r;

	}

	bool Matrix::columns_linearly_independent() {

		if (columns > rows)
			return false;

		// Rows -> Columns    
		transpose();

		Matrix test = gaussian_elimination();

		// Back Columns -> Rows
		transpose();

		bool a_row_is_zero = false;

		for (int xloc = 0; xloc < test.rows; ++xloc) {
			bool row_is_zero = true;
			for (int yloc = 0; yloc < test.columns; yloc++)
				if (test(xloc, yloc) != 0) {
					row_is_zero = false;
					break;
				}
			if (row_is_zero) {
				a_row_is_zero = true;
				break;
			}
		}

		return !a_row_is_zero;

	}

	bool Matrix::rows_linearly_independent() {

		if (rows > columns)
			return false;

		Matrix test = gaussian_elimination();

		bool a_row_is_zero = false;

		for (int xloc = 0; xloc < test.rows; ++xloc) {
			bool row_is_zero = true;
			for (int yloc = 0; yloc < test.columns; yloc++)
				if (test(xloc, yloc) != 0) {
					row_is_zero = false;
					break;
				}
			if (row_is_zero) {
				a_row_is_zero = true;
				break;
			}
		}

		return !a_row_is_zero;
	}

	// LU factorization with Gaussian Elimination and Partial Pivoting
	void Matrix::get_lupp(Matrix& l, Matrix& u, Matrix& p, PermutationFormat pf = PF_MATRIX) const {

		if (!is_square())
			throw(MatrixNotSquareException());

		// Initialize determinant
		long double determinant = 1;

		// Initialize passed u
		u = *this;					                                            // u will evolve from the original matrix

		// Resize passed l
		l.resize(rows, columns);                                                // total l
		l.set_identity();

		// Initialize permutation matrix (if needed)
		if (pf == PF_MATRIX) {                                                  // total p
			p.resize(rows, columns);
			p.set_identity();
		}

		// p_vector and o_vector for efficient permutation handling
		AnonymousMatrix p_vector(rows, 1);
		AnonymousMatrix o_vector(rows, 1);

		// Initialize p_vector, o_vector
		for (int xloc = 0; xloc < rows; ++xloc) {
			p_vector(xloc) = xloc;
			o_vector(xloc) = xloc;
		}


		for (int yloc = 0; yloc < columns; ++yloc) {

			// Reset tem and tpm elements
			Matrix tem(rows, 1, 0);
			tem(yloc) = 1;

			// Write tpm:
			//   *  find absolute maximum element j in column i
			//   *  swap row i with row j in p and u, swap columns in l

			int column_max_position = yloc;
			long double max = u(column_max_position, yloc);

			// Partial pivoting process
			for (int xloc = yloc; xloc < rows; ++xloc)
				if (fabs(u(xloc, yloc)) > fabs(max)) {
					column_max_position = xloc;
					max = u(xloc, yloc);
				}

			// If matrix is not singular proceed ..
			if (max == 0)
				throw (MatrixIsSingularException());

			// Update U and P with TPM only if necessary
			if (yloc != column_max_position) {

				// Update determinant sign
				determinant = -determinant;

				// Effects of permutation on l and u
				u.swap_rows(yloc, column_max_position);
				l.swap_columns(yloc, column_max_position);

				// Effects on permutation on p and p_vector
				if (pf == PF_MATRIX)
					p.swap_rows(yloc, column_max_position);
				p_vector.swap_rows(yloc, column_max_position);

				// If we're returning the PF_MATRIX set its determinant
				if (pf == PF_MATRIX)
					p.set_determinant(determinant);
			}

			// Write tem vector for current column
			for (int xloc = yloc + 1; xloc < rows; ++xloc)
				tem(xloc) = -(u(xloc, yloc) / max);

			// Optimization of l * tem that takes into account the shape
			// of l and tem
			for (int xloc = 0; xloc < rows; ++xloc) {
				register long double inv_product = l(xloc, yloc);   // because tem(j,0) == 1

				for (int k = yloc + 1; k < columns; ++k)
					inv_product += l(xloc, k) * -tem(k);

				l(xloc, yloc) = inv_product;
			}

			// Optimization of tem * u that takes into account the
			// shape of tem and u
			for (int xloc = yloc + 1; xloc < rows; ++xloc) {
				Matrix r(1, columns, 0);

				for (int o = yloc; o < columns; ++o)
					r(o) = tem(xloc) * u(yloc, o) + u(xloc, o);

				for (int k = yloc; k < columns; ++k)
					u(xloc, k) = r(k);
			}
		}

		// Optimized way to calculate p * l, a permutation vector
		// is used to swap the rows of l        
		for (int xloc = 0; xloc < rows; ++xloc)
			while (p_vector(xloc) != o_vector(xloc)) {
				int k = xloc + 1;
				while (p_vector(k) != o_vector(xloc))
					k++;

				o_vector.swap_rows(xloc, k);
				l.swap_rows(xloc, k);
			}


		// Return PF_VECTOR in p
		if (pf == PF_VECTOR)
			p = p_vector;

		// Compute and set determinant        
		for (int xloc = 0; xloc < rows; ++xloc)
			determinant *= u(xloc, xloc);

		determinant_up_to_date = true;

	}

	void Matrix::get_inverse(Matrix& inverse) const {

		if (!is_square())
			throw(MatrixNotSquareException());

		// Adjust inverse size
		inverse.resize(rows, columns);

		// Temporary matrices to hold factorization products
		Matrix l_inverse, u_inverse, p_vector;

		// Compute and store LUPP
		get_lupp(l_inverse, u_inverse, p_vector, PF_VECTOR);

		// Set original permutation vector
		Matrix o_vector(rows, 1);
		for (int xloc = 0; xloc < rows; ++xloc)
			o_vector(xloc) = xloc;

		// Copy transposed l
		l_inverse.transpose();

		// Set reciprocals on the diagonal of u, useless in l since they are ones
		for (int xloc = 0; xloc < rows; ++xloc)
			u_inverse(xloc, xloc) = 1 / u_inverse(xloc, xloc);

		// Calculate inverse of l
		for (int xloc = 1; xloc < rows; ++xloc)
			for (int yloc = xloc - 1; yloc >= 0; --yloc) {
				register long double dot_product = 0;
				for (int k = xloc; k > 0; --k)
					dot_product += l_inverse(xloc, k) * l_inverse(yloc, k);
				l_inverse(xloc, yloc) = -dot_product;                                 // Optimization of dot_product * - l_inverse.at(j,j)
			}

		// Set zeroes on the upper half of l^-1
		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = xloc + 1; yloc < columns; ++yloc)
				l_inverse(xloc, yloc) = 0;

		// Calculate inverse of u
		for (int xloc = 1; xloc < rows; ++xloc)
			for (int yloc = xloc - 1; yloc >= 0; --yloc) {
				register long double dot_product = 0;
				for (int k = xloc; k > 0; --k) {
					dot_product += u_inverse(xloc, k) * u_inverse(yloc, k);
				}
				u_inverse(xloc, yloc) = dot_product * -u_inverse(yloc, yloc);
			}

		u_inverse.transpose();


		// Set zeroes on the lower half of u^-1
		for (int yloc = 0; yloc < columns; ++yloc)
			for (int xloc = yloc + 1; xloc < rows; ++xloc)
				u_inverse(xloc, yloc) = 0;



		// Optimization of u^-1 * l^-1 that takes into account the
		// shape of the two matrices
		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < columns; ++yloc)
				for (int h = columns - 1; h >= std::min(xloc, yloc); --h)
					inverse(xloc, yloc) += u_inverse(xloc, h) * l_inverse(h, yloc);

		// Smart way to translate a row permutation vector to obtain
		// a column permutation vector
		Matrix p_vector_t(rows, 1);
		for (int xloc = 0; xloc < rows; ++xloc)
			p_vector_t(p_vector(xloc)) = xloc;

		// Optimization of columns permutation
		for (int xloc = 0; xloc < rows; ++xloc)
			while (p_vector_t(xloc) != o_vector(xloc)) {
				int k = xloc + 1;
				while (p_vector_t(k) != o_vector(xloc))
					k++;

				o_vector.swap_rows(xloc, k);
				inverse.swap_columns(xloc, k);
			}
	}

	void Matrix::get_inverse_with_column(Matrix const& old_inverse,
		Matrix const& new_column,
		int q,
		Matrix& new_inverse) {

		// Prepare result
		new_inverse.resize(old_inverse.rows, old_inverse.columns);
		Matrix a_tilde(old_inverse * new_column);

		for (int xloc = 0; xloc < old_inverse.rows; ++xloc)
			for (int yloc = 0; yloc < old_inverse.columns; ++yloc)
				if (xloc != q)
					new_inverse(xloc, yloc) = old_inverse(xloc, yloc) - ((old_inverse(q, yloc) * a_tilde(xloc)) / a_tilde(q));
				else
					new_inverse(xloc, yloc) = old_inverse(q, yloc) / a_tilde(q);


	}


	void Matrix::solve(Matrix& x, Matrix const& b) const {

		// Invert L and U
		Matrix l_inverse, u_inverse, p_vector;

		// Calculate LUPP factorization
		get_lupp(l_inverse, u_inverse, p_vector, PF_VECTOR);

		// Copy transposed l
		l_inverse.transpose();

		// Set reciprocals on the diagonal of u (useless in l since diagonal elements are ones)
		for (int xloc = 0; xloc < rows; ++xloc)
			u_inverse(xloc, xloc) = 1 / u_inverse(xloc, xloc);

		// Calculate inverse of l
		for (int xloc = 1; xloc < rows; ++xloc)
			for (int yloc = xloc - 1; yloc >= 0; --yloc) {
				register long double dot_product = 0;
				for (int k = xloc; k > 0; --k)
					dot_product += l_inverse(xloc, k) * l_inverse(yloc, k);
				l_inverse(xloc, yloc) = -dot_product;                                 // Optimization due to ones on diagonal
			}

		// Set zeroes on the upper half of l^-1
		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = xloc + 1; yloc < columns; ++yloc)
				l_inverse(xloc, yloc) = 0;

		// Calculate inverse of u
		for (int xloc = 1; xloc < rows; ++xloc)
			for (int yloc = xloc - 1; yloc >= 0; --yloc) {
				register long double dot_product = 0;
				for (int k = xloc; k > 0; --k) {
					dot_product += u_inverse(xloc, k) * u_inverse(yloc, k);
				}
				u_inverse(xloc, yloc) = dot_product * -u_inverse(yloc, yloc);
			}

		u_inverse.transpose();

		// Set zeroes on the lower half of u^-1
		for (int yloc = 0; yloc < columns; ++yloc)
			for (int xloc = yloc + 1; xloc < rows; ++xloc)
				u_inverse(xloc, yloc) = 0;

		// Optimization of p * b
		Matrix pb(rows, 1);
		for (int xloc = 0; xloc < rows; ++xloc)
			pb(xloc) = b(p_vector(xloc));

		// Set x shape	
		x.resize(rows, 1);

		// Optimization of x = l_inverse * pb;
		for (int xloc = 0; xloc < rows; ++xloc) {
			register long double dot_product = pb(xloc);
			for (int yloc = 0; yloc < xloc; ++yloc) {
				dot_product += l_inverse(xloc, yloc) * pb(yloc);
			}
			x(xloc) = dot_product;
		}

		// Optimization of x = u_inverse * x
		for (int xloc = 0; xloc < rows; ++xloc) {
			register long double dot_product = 0;
			for (int yloc = columns - 1; yloc >= xloc; --yloc)
				dot_product += u_inverse(xloc, yloc) * x(yloc);
			x(xloc) = dot_product;
		}
	}

	/*
	Matrix::storage_accessor
	*/
	Matrix::storage_accessor::storage_accessor(long double& dest, Matrix& parent) :
		dest(dest),
		parent(parent) {
	}

	Matrix::storage_accessor::operator long double const& () const {
		return dest;
	}

	Matrix::storage_accessor& Matrix::storage_accessor::operator=(Matrix::storage_accessor& new_value) {

		if (new_value.dest == dest)
			return *this;

		dest = new_value.dest;

		// Update bools
		parent.determinant_up_to_date = false;
		parent.inverse_up_to_date = false;
		parent.lu_up_to_date = false;

		return *this;
	}

	Matrix::storage_accessor& Matrix::storage_accessor::operator=(long double const& new_value) {

		if (new_value == dest)
			return *this;

		dest = new_value;

		// Update bools
		parent.determinant_up_to_date = false;
		parent.inverse_up_to_date = false;
		parent.lu_up_to_date = false;

		return *this;
	}

	/*
	Matrix::storage
	===============
	Reference counted vector of long doubles.

	*/

	Matrix::storage::storage(int size) :
		contents(new std::vector< long double>(size)),
		counter(1) {
	}

	Matrix::storage::storage(Matrix::storage& s) :
		contents(new std::vector< long double>(*(s.contents))),
		counter(1) {
	}

	Matrix::storage::storage(int size, long double value) :
		contents(new std::vector< long double>(size, value)),
		counter(1) {
	}

	long double& Matrix::storage::at(int pos) {
		return contents->at(pos);
	}

	Matrix::storage::~storage() {
		// Decrease reference count
		--counter;

		// Reference count goes to zero
		if (counter == 0) {
			delete contents;
		}
	}


	/*
	AnonymousMatrix
	===============
	Matrix that is generate from scratch during the calculations.

	*/

	AnonymousMatrix AnonymousMatrix::operator*(Matrix const& m) {

		if (columns != m.rows)
			throw(SizeMismatchException());

		// Allocate return matrix filled with zeroes
		AnonymousMatrix r(rows, m.columns);

		for (int xloc = 0; xloc < rows; ++xloc)
			for (int yloc = 0; yloc < m.columns; ++yloc)
				for (int h = 0; h < columns; ++h)
					r(xloc, yloc) += at(xloc, h) * m(h, yloc);

		// Swap pointers
		rows = r.rows;
		columns = r.columns;
		swap(values->contents, r.values->contents);

		return *this;
	}

	AnonymousMatrix::AnonymousMatrix(const AnonymousMatrix& m) : Matrix(m.rows, m.columns) {

		swap(Matrix::values->contents, m.values->contents);

	}

	// Other constructors	
	AnonymousMatrix::AnonymousMatrix(const Matrix& m) : Matrix(m) {	}
	AnonymousMatrix::AnonymousMatrix(int r, int c) : Matrix(r, c) { }

	// Auxiliary 

	bool tol_equal(long double n, long double m, long double tol = 0.0000000000000001) {
		if (abs(n - m) > tol)
			return false;
		return true;
	}



}