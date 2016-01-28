//
//  QuadTreeNode.h
//
//  Created by Tomas Basham on 15/03/2014.
//  Copyright (c) 2014 tomasbasham.co.uk. All rights reserved.
//

#ifndef Buildings_QuadTreeNode_h
#define Buildings_QuadTreeNode_h

class QuadTreeNodeData;
class QuadTreeNode;

typedef void(^QuadTreeNodeDataBlock)(QuadTreeNodeData data);
typedef void(^QuadTreeNodeTraverseBlock)(QuadTreeNode *node);

/**
 * QuadTreeNodeData holds the latitude and longitude of the spatial node.
 * The data field is a void pointer to arbitary data. This will hold the
 * Obj-c object to be cast when necessary.
 */
typedef struct QuadTreeNodeData {
    double latitude;
    double longitude;
    void *data;
} QuadTreeNodeData;

QuadTreeNodeData QuadTreeNodeDataMake(double latitude, double longitude, void *data);

/**
 * BoundingBox describes a box whoose bounds contain spactial nodes between
 * them. This is used to determine where nodes will be placed and for
 * querying nodes.
 */
typedef struct BoundingBox {
    double x0; double y0;
    double x1; double y1;
} BoundingBox;

BoundingBox BoundingBoxMake(double x0, double y0, double x1, double y1);

bool BoundingBoxContainsData(BoundingBox boundary, QuadTreeNodeData data);
bool BoundingBoxIntersectsBoundingBox(BoundingBox boundary, BoundingBox test);

/**
 * QuadTreeNode describes a spatial node in the tree.
 */
class QuadTreeNode {
protected:
    QuadTreeNode *northWest;
    QuadTreeNode *northEast;
    QuadTreeNode *southWest;
    QuadTreeNode *southEast;
    QuadTreeNodeData *points;
    BoundingBox boundary;
    int capacity;
    int count;
public:
    QuadTreeNode(BoundingBox boundary, int capacity);
    ~QuadTreeNode();
    
    static QuadTreeNode* QuadTreeNodeMake(BoundingBox boundary, int capacity);
    
    void gatherDataWithinBoundary(BoundingBox boundary, QuadTreeNodeDataBlock block);
    void traverse(QuadTreeNodeTraverseBlock block);
    bool insert(QuadTreeNodeData data);
    void subdivide();
    
    QuadTreeNodeData* getPoints() const;
    BoundingBox getBoundary() const;
    int getCapacity() const;
};

class QuadTree : public QuadTreeNode {
public:
    QuadTree(QuadTreeNodeData *data, BoundingBox bourdary, int capacity);
    static QuadTree* QuadTreeMake(QuadTreeNodeData *data, BoundingBox boundary, int capacity);
};

#endif