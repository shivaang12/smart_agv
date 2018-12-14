/********************************************************************
 *   MIT License
 *  
 *   Copyright (c) 2018 Shivang Patel
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

/** @file location.hpp
 *  @brief Definition of class Location
 *
 *  This file contains definitions of class Location which performs
 *  provides custom data type which stores cordinates and orientation
 *  of the robot.
 * 
 *  @author Shivang Patel
 */


#pragma once
#include <string>

class Location {
 public:
   std::string loc;
   double pointX;
   double pointY;
   double pointZ;
   double orientationX;
   double orientationY;
   double orientationZ;
   double orientationW;
   Location(std::string s, double px, double py, double pz, double ox,
            double oy, double oz, double ow)
       : loc(s), pointX(px), pointY(py), pointZ(pz), orientationX(ox),
         orientationY(oy), orientationZ(oz), orientationW(ow) {}
};