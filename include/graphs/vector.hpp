#pragma once

namespace linalg
{
    class Vector
    {
    public:
        // properties
        float x, y;

        // constructer
        Vector(float x = 0.f, float y = 0.f);

        // methods
        Vector unit() const;                   // returns unit vector
        float magnitude() const;               // returns magnitude of a vector
        float dot(const Vector &vector) const; // return dot product of two vectors

        // operator overloading
        Vector operator+(const Vector &vector) const; // adding
        Vector operator-(const Vector &vector) const; // subtracting
        Vector operator*(float scalar) const;         // multiplying
        Vector operator/(float scalar) const;         // dividing
    };
}