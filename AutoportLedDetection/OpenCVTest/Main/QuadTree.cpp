//
//  QuadTreeNode.cpp
//
//  Created by Tomas Basham on 15/03/2014.
//  Copyright (c) 2014 tomasbasham.co.uk. All rights reserved.
//

#include <iostream>
#include "QuadTree.h"

/// <summary>
/// Create an instance of a QuadTreeNodeData struct and fill it
/// with data.
/// </summary>
/// <param name="latitude">the latitude of a spatial point</param>
/// <param name="longitude">the longitude of a spatial point</param>
/// <param name="data">an object that can hold any arbitrary pertaining to a spatial point</param>
/// <returns>a QuadTreeNodeData struct containing the latitude, longitude, and any arbitrary data pertaining to a spatial point</returns>
QuadTreeNodeData QuadTreeNodeDataMake(double latitude, double longitude, void *data) {
    
    QuadTreeNodeData nodeData;
    nodeData.latitude = latitude;
    nodeData.longitude = longitude;
    nodeData.data = data;
    
    return nodeData;
    
}

/**
 * Create an instance of a BoundingBox struct and fill it with 
 * data.
 *
 * @param x0 top left x coordinate of a boundary.
 * @param y0 top left y coordinate of a boundary.
 * @param x1 bottom right x coordinate of a boundary.
 * @param y1 bottom right y coordinate of a boundary.
 * @return a BoundingBox struct representing a spatial boundary.
 */
BoundingBox BoundingBoxMake(double x0, double y0, double x1, double y1) {
    
    BoundingBox boundingBox;
    boundingBox.x0 = x0;
    boundingBox.y0 = y0;
    boundingBox.x1 = x1;
    boundingBox.y1 = y1;
    
    return boundingBox;
    
}

/**
 * Check if the node data is contained within the boundary.
 *
 * @param boundary a spatial boundary.
 * @param data a QuadTreeNodeData struct containing data to be queried.
 * @return true if the boundary contains a spatial point with data that fits within it, otherwise false.
 */
bool BoundingBoxContainsData(BoundingBox boundary, QuadTreeNodeData data) {
    
    bool containsX = boundary.x0 <= data.latitude && data.latitude <= boundary.x1;
    bool containsY = boundary.y0 <= data.longitude && data.longitude <= boundary.y1;
    
    return containsX && containsY;
    
}

/**
 * Check if 2 boundaries intersect.
 *
 * @param boundary a spatial boundary.
 * @param test a spatial boundary to be queried.
 * @return true if the boundaries intersect, otherwise false.
 */
bool BoundingBoxIntersectsBoundingBox(BoundingBox boundary, BoundingBox test) {
    
    return (boundary.x0 <= test.x1 && boundary.x1 >= test.x0 && boundary.y0 <= test.y1 && boundary.y1 >= test.y0);
    
}

/**
 * Create an instance of a QuadTreeNode and fill it with data.
 *
 * @param boundary a spatial boundary.
 * @param capacity the total number of points this node can hold before it is full.
 */
QuadTreeNode::QuadTreeNode(BoundingBox boundary, int capacity) {
    
    this->northWest = NULL;
    this->northEast = NULL;
    this->southWest = NULL;
    this->southEast = NULL;
    
    this->boundary = boundary;
    this->capacity = capacity;
    this->count = 0;
    this->points = (QuadTreeNodeData *)malloc ( sizeof ( QuadTreeNodeData ) * capacity );
    
}

/**
 * Free up node memory. This, recursively frees up sub-nodes and
 * bubbles up the QuadTree.
 */
QuadTreeNode::~QuadTreeNode() {
    
    // Free all child nodes.
    if ( this->northWest != NULL ) delete this->northWest;
    if ( this->northEast != NULL ) delete this->northEast;
    if ( this->southWest != NULL ) delete this->southWest;
    if ( this->southEast != NULL ) delete this->southEast;
    
    // Free all nodes contained within this node.
    for ( int i = 0; i < this->count; i++ ) {
        free ( this->points[i].data );
    }
    
    // Free this node.
    delete this->points;
    
}

/**
 * Create an instance of a QuadTreeNode and fill it with data.
 * This is a class method.
 *
 * @param boundary a spatial boundary.
 * @param capacity the total number of points this node can hold before it is full.
 * @return a QuadTreeNode that makes up a part of the overall QuadTree.
 */
QuadTreeNode* QuadTreeNode::QuadTreeNodeMake(BoundingBox boundary, int capacity) {
    
    QuadTreeNode *node = new QuadTreeNode ( boundary, capacity );	
    return node;
    
}

/**
 * Gathers all node data within a boundary and passes it to a
 * block of code to be done with what will.
 *
 * @param boundary a spatial boundary.
 * @param block gathered data is passed to this block.
 */
void QuadTreeNode::gatherDataWithinBoundary(BoundingBox boundary, QuadTreeNodeDataBlock block) {
    
    // If range is not contained in the node's boundingBox then bail
    if ( !BoundingBoxIntersectsBoundingBox ( this->boundary, boundary ) ) {
        return;
    }
    
    for ( int i = 0; i < this->count; i++ ) {
        // Gather points contained in range
        if ( BoundingBoxContainsData ( boundary, this->points[i] ) ) {
            block ( this->points[i] );
        }
    }
    
    // Bail if node is leaf
    if ( this->northWest == NULL ) {
        return;
    }
    
    // Otherwise traverse down the tree
    this->northWest->gatherDataWithinBoundary ( boundary, block );
    this->northEast->gatherDataWithinBoundary ( boundary, block );
    this->southWest->gatherDataWithinBoundary ( boundary, block );
    this->southEast->gatherDataWithinBoundary ( boundary, block );
    
}

/**
 * Run a block of code over each node, recursively.
 *
 * @param block this block is run for each QuadTreeNode traversed.
 */
void QuadTreeNode::traverse(QuadTreeNodeTraverseBlock block) {
    
    block ( this );
    
    if ( this->northWest == NULL ) {
        return;
    }
    
    this->northWest->traverse ( block );
    this->northEast->traverse ( block );
    this->southWest->traverse ( block );
    this->southEast->traverse ( block );
    
}

/**
 * Insert a node into the QuadTree. If the current node is not
 * yet at capacity then add the new node to it's points array.
 * Otherwise check, sequentially, if the new node can be added
 * to any of the current nodes four quadrants.
 *
 * @param data a QuadTreeNodeData struct containing the data to be inserted.
 * @return true if the data was successfully inserted, otherwise false.
 */
bool QuadTreeNode::insert(QuadTreeNodeData data) {
    
    // Return if our coordinate is not in the boundingBox
    if ( !BoundingBoxContainsData ( this->boundary, data ) ) {
        return false;
    }
    
    // Add the coordinate to the points array.
    if ( this->count < this->capacity ) {
        this->points[this->count++] = data;
        return true;
    }
    
    // Check to see if the current node is a leaf, if it is, split.
    if ( this->northWest == NULL ) {
        this->subdivide();
    }
    
    // Traverse the tree
    if ( this->northWest->insert ( data ) ) return true;
    if ( this->northEast->insert ( data ) ) return true;
    if ( this->southWest->insert ( data ) ) return true;
    if ( this->southEast->insert ( data ) ) return true;
    
    // Default. Was unable to add the node.
    return false;
    
}

/**
 * Divide a quadrent into four equal parts ready for further
 * insertion of nodes.
 */
void QuadTreeNode::subdivide() {
    
    BoundingBox box = this->boundary;
    
    // Spit the quadrant into four equal parts.
    double xMid = (box.x1 + box.x0) / 2.0;
    double yMid = (box.y1 + box.y0) / 2.0;
    
    // Create the north west bounding box.
    BoundingBox northWest = BoundingBoxMake ( box.x0, box.y0, xMid, yMid );
    this->northWest = new QuadTreeNode ( northWest, this->capacity );
    
    // Create the north east bounding box.
    BoundingBox northEast = BoundingBoxMake ( xMid, box.y0, box.x1, yMid );
    this->northEast = new QuadTreeNode ( northEast, this->capacity );
    
    // Create the south west bounding box.
    BoundingBox southWest = BoundingBoxMake ( box.x0, yMid, xMid, box.y1 );
    this->southWest = new QuadTreeNode ( southWest, this->capacity );
    
    // Create the south east bounding box.
    BoundingBox southEast = BoundingBoxMake ( xMid, yMid, box.x1, box.y1 );
    this->southEast = new QuadTreeNode ( southEast, this->capacity );
    
}

/**
 * Return the array of spatial points within this node.
 *
 * @return an array of spatial points.
 */
QuadTreeNodeData* QuadTreeNode::getPoints() const {
    
    return this->points;
    
}

/**
 * Return the spatial boundary of this node.
 *
 * @return a BoundingBox struct representing a spatial boundary.
 */
BoundingBox QuadTreeNode::getBoundary() const {
    
    return this->boundary;
    
}

/**
 * Return the total capacity of this node.
 *
 * @return the total capacity of this node.
 */
int QuadTreeNode::getCapacity() const {
    
    return this->capacity;
    
}

QuadTree::QuadTree(QuadTreeNodeData *data, BoundingBox bourdary, int capacity) : QuadTreeNode ( boundary, capacity ) {
    
    for ( int i = 0; i < sizeof ( data ) / sizeof ( QuadTreeNodeData ); i++ ) {
        this->insert ( data[i] );
    }
    
}

/**
 * Build the QuadTree by passing an array of QuadTreeNodeData
 * structs which will be inserted sequentially untill the tree
 * is at capacity or the data is exhausted.
 *
 * @param data an array of QuadTreeNodeData structs.
 * @param boundary a spatial bounday.
 * @param capacity the total number of points a node can hold before it is full.
 * @return the top most QuadTreeNode. This obviously is the root node.
 */
QuadTreeNode* QuadTreeMake(QuadTreeNodeData *data, BoundingBox boundary, int capacity) {
    
    QuadTree *tree = new QuadTree ( data, boundary, capacity );
    return tree;
    
}