/**
 * This file is part of λ-BRIEF Localization
 *
 * Copyright (c) 2020 Ricardo Westhauser <rswesthauser at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https://github.com/rswesthauser/Lambdabrief-Localization>
 *
 * λ-BRIEF Localization is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * λ-BRIEF Localization is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with λ-BRIEF Localization.  If not, see <https://www.gnu.org/licenses/>.
**/

#ifndef VEGETATONINDEXGEN_H
#define VEGETATONINDEXGEN_H

#include "src/VegetationIndex/VegetationIndex.h"
#include "src/VegetationIndex/Ndvi.h"
#include "src/VegetationIndex/Grvi.h"
#include "config.h"

class VegetatonIndexGen
{
public:    
    VegetatonIndexGen();
    static VegetationIndex* InitVegetationIndex();
};
#endif // VEGETATONINDEXGEN_H
