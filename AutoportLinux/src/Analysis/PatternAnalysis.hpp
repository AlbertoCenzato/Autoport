/*==============================================================================
Software for Autoport project

// Copyright   : Copyright (c) 2016, Alberto Cenzato
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
//============================================================================ */

#ifndef PATTERN_ANALYSIS_HPP_
#define PATTERN_ANALYSIS_HPP_

//#include "../Utils/global_includes.hpp"

class LedDescriptor;

/*
 * This class decides if a given vector of 2D points represents
 * the LED pattern or not and, if yes, reorders the points properly
 * as described by the pattern documentation (see Dropbox > AUTOPORT >
 * SENSORI MATERIALE > pattern.docx, "ottava geometria")
 *
 * TODO: this class should be abstract, leaving the implementation of
 * 		 firstPhase and secondPhase to its concrete classes.
 */

class PatternAnalysis {

public:
	PatternAnalysis();
	~PatternAnalysis();

	/*
	 * Receives the descriptors and states if they represent a pattern or not.
	 * If they match the expected pattern they are reordered accordingly
	 * to the pattern order.
	 * Uses two different matching algorithms: if t = 0 uses firstPhase,
	 * otherwise uses nearestPoints.
	 *
	 * @descriptors: descriptors to evaluate
	 * @return: true if a match is found, false otherwise
	 */
	bool evaluate(std::vector<LedDescriptor> &descriptors);

private:
	std::vector<LedDescriptor> oldDescriptors; // points at time t-1
	std::vector<cv::Point2f>   pattern;	 	   // model of the pattern
	float maxDistance   = 75;
	int   minNumOfMatch = 4;

	/*
	 * Receives the descriptors and states if they represent
	 * a pattern or not, using an algorithm specific for the pattern.
	 * Used at time t = 0 (the very first frame) or when the drone
	 * has lost track of the LEDs.
	 *
	 * @descriptors: descriptors to evaluate.
	 * @return: true if a match is found, false otherwise.
	 *
	 * TODO: should return the number of matched descriptors
	 * 		 as PatternAnalysis::secondPhase() does.
	 */
	bool firstPhase(std::vector<LedDescriptor> &descriptors);

	/*
	 * Receives the descriptors and states if they represent
	 * a pattern or not, comparing them to "oldPoints".
	 * Used at time t > 0.
	 *
	 * @descriptors: descriptors to evaluate
	 * @return: the number of matched descriptors
	 */
	int secondPhase(std::vector<LedDescriptor> &descriptors);


	/**
	 * Find the element of the set nearest to the specified descriptor.
	 * The algorithm used is a simple linear search on the set using
	 * an L2 metric in the 6D space of the descriptors. Use only for
	 * small sets.
	 *
	 * @oldDescriptor: reference descriptor.
	 * @descriptors: set to look into.
	 * @return: index of the nearest descriptor in the set.
	 */
	int findNearestPoint(const LedDescriptor &oldDescriptor,
						 const std::vector<LedDescriptor> &descriptors);

};

#endif
