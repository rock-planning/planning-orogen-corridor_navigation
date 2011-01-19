/* Generated from orogen/lib/orogen/templates/typekit/Opaques.cpp */

#include "Opaques.hpp"
#include <base/typekit/OpaqueConvertions.hpp>

    /** Returns the intermediate value that is contained in \c real_type */
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    /** Release ownership of \c real_type on the corresponding intermediate
     * pointer.
     */


void orogen_typekits::toIntermediate(::wrappers::vfh_star::Tree& intermediate, ::vfh_star::Tree const& real_type)
{
    std::list<vfh_star::TreeNode> const& nodes = real_type.getNodes();

    intermediate.nodes.clear();

    for (std::list<vfh_star::TreeNode>::const_iterator it = nodes.begin();
            it != nodes.end(); ++it)
    {
        wrappers::vfh_star::TreeNode wrapped_node;

        vfh_star::TreeNode const* node = &(*it);

        toIntermediate(wrapped_node.pose, node->getPose());
        wrapped_node.parent    = node->getParent()->getIndex();
        wrapped_node.cost      = node->getCost();
        wrapped_node.heuristic = node->getHeuristic();
        wrapped_node.direction = node->getDirection();
        wrapped_node.positionTolerance = node->getPositionTolerance();
        wrapped_node.headingTolerance  = node->getHeadingTolerance();
        intermediate.nodes.push_back(wrapped_node);
    }

    if (real_type.getFinalNode())
        intermediate.finalNode = real_type.getFinalNode()->getIndex();
    else
        intermediate.finalNode = -1;
}

void orogen_typekits::fromIntermediate(::vfh_star::Tree& real_type, ::wrappers::vfh_star::Tree const& intermediate)
{
    std::vector<vfh_star::TreeNode*> index_to_node;
    real_type.clear();

    std::vector<wrappers::vfh_star::TreeNode> const& nodes = intermediate.nodes;
    index_to_node.reserve(nodes.size());
    for (std::vector<wrappers::vfh_star::TreeNode>::const_iterator it = nodes.begin();
            it != nodes.end(); ++it)
    {
        wrappers::vfh_star::TreeNode const& wrapped_node = *it;
        base::Pose p;
        fromIntermediate(p, wrapped_node.pose);
        vfh_star::TreeNode* node;
        if (it == nodes.begin())
            node = real_type.createRoot(p, wrapped_node.direction);
        else
            node = real_type.createChild(index_to_node[wrapped_node.parent], p, wrapped_node.direction);

        node->setCost(wrapped_node.cost);
        node->setHeuristic(wrapped_node.heuristic);
        node->setPositionTolerance(wrapped_node.positionTolerance);
        node->setHeadingTolerance(wrapped_node.headingTolerance);
        index_to_node.push_back(node);
    }
    if (intermediate.finalNode != -1)
        real_type.setFinalNode(index_to_node[intermediate.finalNode]);
    else
        real_type.setFinalNode(0);
}

