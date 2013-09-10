/* Generated from orogen/lib/orogen/templates/typekit/Opaques.cpp */

#include "Opaques.hpp"
#include <base/typekit/OpaqueConvertions.hpp>

    /** Returns the intermediate value that is contained in \c real_type */
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    /** Release ownership of \c real_type on the corresponding intermediate
     * pointer.
     */

void addRecursive(::wrappers::vfh_star::Tree& intermediate, const ::vfh_star::TreeNode *node, const vfh_star::TreeNode *finalNode, int &nodeId)
{    
    if(node == finalNode)
    {
        intermediate.finalNode = nodeId;
    }
    
    wrappers::vfh_star::TreeNode wrapped_node;
    orogen_typekits::toIntermediate(wrapped_node.pose, node->getPose());
    wrapped_node.nodeId    = nodeId;
    wrapped_node.cost      = node->getCost();
    wrapped_node.heuristic = node->getHeuristic();
    wrapped_node.direction = node->getDirection();
    wrapped_node.positionTolerance = node->getPositionTolerance();
    wrapped_node.headingTolerance  = node->getHeadingTolerance();
    
    nodeId++;
    
    for(std::vector< ::vfh_star::TreeNode *>::const_iterator it = node->getChildren().begin();
        it != node->getChildren().end(); it++)
    {
        //this is the id of the next added child
        wrapped_node.childs.push_back(nodeId);
        addRecursive(intermediate, *it, finalNode, nodeId);
        
    }
    intermediate.nodes.push_back(wrapped_node);
};

void orogen_typekits::toIntermediate(::wrappers::vfh_star::Tree& intermediate, ::vfh_star::Tree const& real_type)
{
    intermediate.nodes.clear();
    intermediate.finalNode = -1;

    const vfh_star::TreeNode *root = real_type.getRootNode();
    const vfh_star::TreeNode *finalNode = real_type.getFinalNode();
    int idCnt = 0;

    if(root)
        addRecursive(intermediate, root, finalNode, idCnt);
}

void orogen_typekits::fromIntermediate(::vfh_star::Tree& real_type, ::wrappers::vfh_star::Tree const& intermediate)
{
    real_type.clear();    
    if(intermediate.nodes.empty())
    {
        return;
    }

    std::vector<vfh_star::TreeNode*> index_to_node;
    std::vector<wrappers::vfh_star::TreeNode> const& nodes = intermediate.nodes;
    index_to_node.resize(nodes.size());
    
    wrappers::vfh_star::TreeNode const& wrapped_root = intermediate.nodes.back();
    assert(wrapped_root.nodeId == 0);
    
    base::Pose p;
    fromIntermediate(p, wrapped_root.pose);
    vfh_star::TreeNode* root_node = real_type.createRoot(p, wrapped_root.direction);

    //construct map and create nodes
    for (std::vector<wrappers::vfh_star::TreeNode>::const_iterator it = nodes.begin();
            it != nodes.end(); ++it)
    {
        wrappers::vfh_star::TreeNode const& wrapped_node = *it;
        base::Pose p;
        fromIntermediate(p, wrapped_node.pose);
        vfh_star::TreeNode* node;
        if (it->nodeId == 0)
            node = root_node;
        else
            node = real_type.createNode(p, wrapped_node.direction);

        node->setCost(wrapped_node.cost);
        node->setHeuristic(wrapped_node.heuristic);
        node->setPositionTolerance(wrapped_node.positionTolerance);
        node->setHeadingTolerance(wrapped_node.headingTolerance);
        index_to_node[it->nodeId] = node;
    }   

    //connect all nodes to a tree
    for (std::vector<wrappers::vfh_star::TreeNode>::const_iterator it = nodes.begin();
        it != nodes.end(); ++it)
    {
        wrappers::vfh_star::TreeNode const& wrappedNode = *it;
        vfh_star::TreeNode *parent = index_to_node[it->nodeId];

        for(std::vector<int>::const_iterator it2 = wrappedNode.childs.begin(); it2 != wrappedNode.childs.end(); it2++)
        {
            parent->addChild(index_to_node[*it2]);
        }

    }
    
    if(intermediate.finalNode != -1)
        real_type.setFinalNode(index_to_node[intermediate.finalNode]);
    else
        real_type.setFinalNode(0);
}

