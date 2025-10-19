#include "../../include/graphs/vector.hpp"
#include <cmath>

linalg::Vector::Vector(float x, float y) : x(x), y(y) {}

// operator overloading

// adds two vectors
linalg::Vector linalg::Vector::operator+(const Vector &vector) const
{
    return Vector(this->x + vector.x, this->y + vector.y);
}

// subtracts two vectors
linalg::Vector linalg::Vector::operator-(const Vector &vector) const
{
    return Vector(this->x - vector.x, this->y - vector.y);
}

// multiplies a vector by a scalar
linalg::Vector linalg::Vector::operator*(float scalar) const
{
    return Vector(this->x * scalar, this->y * scalar);
}

// divides a vector by a scalar
linalg::Vector linalg::Vector::operator/(float scalar) const
{
    return Vector(this->x / scalar, this->y / scalar);
}

// returns the unit vector
linalg::Vector linalg::Vector::unit() const
{
    float mag = this->magnitude();
    if(mag == 0){
        return linalg::Vector(0.f, 0.f);
    }

    return (*this) / magnitude();
}

// returns the magnitude of a vector
float linalg::Vector::magnitude() const
{
    return std::sqrt(std::pow(this->x, 2) + std::pow(this->y, 2));
}

// returns the dot products of two vectors
float linalg::Vector::dot(const Vector &vector) const
{
    return this->x * vector.x + this->y * vector.y;
}
