// CamVector.cpp
#include "CamVector.h"
#include "CamDirection.h"

namespace PathForge {

CamVector::operator CamDirection() const {
    return CamDirection(*this);
}

} // namespace PathForge