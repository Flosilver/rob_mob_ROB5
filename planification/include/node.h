#ifndef __NODE_H__
#define __NODE_H__

#include <memory>
#include <vector>
#include <cstdlib>

#include <opencv2/highgui.hpp>

class Node
{
    int x_;
    int y_;
    std::shared_ptr<Node> parent_;
    std::vector<Node> childs_;

public:
    Node(int x = 0, int y = 0, std::shared_ptr<Node> parent = nullptr);
    Node(const Node& n);
    ~Node();
    static Node &newRandNode(const cv::Mat &env);

    const int &x() const { return x_; }
    int x() { return x_; }

    const int &y() const { return y_; }
    int y() { return y_; }

    std::shared_ptr<const Node> parent() const { return parent_; }
    std::shared_ptr<Node> parent() { return parent_; }

    const std::vector<Node> childs() const { return childs_; }
    std::vector<Node> childs() { return childs_; }

    float norm(const Node &to);
    void addChild(Node& n);
};


#endif // __NODE_H__