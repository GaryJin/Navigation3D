#ifndef NODE
#define NODE
#include <stdlib.h>
#include <list>


/**
 * Base class for an RRT tree node.
 *
 * @param T The datatype representing the state in the space the RRT
 * will be searching.
 */
template<typename T>
class Node {
public:
    Node(const T &state, Node<T> *parent = nullptr) {
        _parent = parent;
        _state = state;
        distance = 0;
        if (_parent) {
            _parent->_children.push_back(this);
        }
    }

    const Node<T> *parent() const{
        return _parent;
    }

    void addchildren(Node<T> *newchildren)
    {
        _children.push_back(newchildren);
    }

    void removechildren(Node<T> *children)
    {
        _children.remove(children);
    }

    void changeparent(Node<T> *newparent)
    {
        _parent->removechildren(this);
        _parent = newparent;
    }

    /**
     * Gets the number of ancestors (parent, parent's parent, etc) that
     * the node has.
     * Returns 0 if it doesn't have a parent.
     */
    int depth() const {
        int n = 0;
        for (Node<T> *ancestor = _parent; ancestor != nullptr; ancestor = ancestor->_parent) {
            n++;
        }
        return n;
    }

    /**
     * set the distance of the Node to the root
     */
    void setDistance(double dist)
    {
        distance = dist;
    }

    void resetDistance(double dist)
    {
        if(!_children.empty ())
            for( Node<T>* iter : _children)
                iter->resetDistance (dist);
        distance -= dist;

    }

    double getDistance()
    {
        return distance;
    }

    /**
     * The @state property is the point in the state-space that this
     * Node represents.  Generally this is a vector (could be 2d, 3d, etc)
     */
    const T &state() const {
        return _state;
    }

private:
    T _state;
    std::list<Node<T> *> _children;
    Node<T> *_parent;
    double distance;
};

#endif // NODE

