/*
    This source file is part of Rigs of Rods
    Copyright 2016 Fabian Killus

    For more information, see http://www.rigsofrods.org/

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "../../ForwardDeclarations.h"

/// Represents a triangle in three-dimensional space.
///
/// Stores the three vertices #a, #b ,#c of the triangle and two
/// (non-unit-length) vectors #u and #v which span the plane within which the
/// triangle lies.
class Triangle
{
public:
    /// Construct triangle from three given vertices.
    explicit Triangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
        : a{a}, b{b}, c{c}, u{a - c}, v{b - c}, m_initialized{false}
    {
    }

    /// Return normal vector of the triangle.
    ///
    /// The normal vector has unit length.
    /// @warning Not thread-safe due to caching implementation!
    glm::vec3 normal() const
    {
        if (!m_initialized)
        {
            m_normal = glm::cross(u, v);
            m_normal = glm::normalize(m_normal);
            m_initialized = true;
        }
        return m_normal;
    }

    const glm::vec3 a, //!< Vertex a
                        b, //!< Vertex b
                        c; //!< Vertex c

    const glm::vec3 u, //!< Span vector u
                        v; //!< Span vector v

private:
    mutable bool m_initialized;
    mutable glm::vec3 m_normal; //!< Cached normal vector
};
