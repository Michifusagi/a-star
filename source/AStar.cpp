#include "AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    // コストを初期化
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

// コンストラクタ実装
AStar::Generator::Generator()
{
    // 斜め移動を無効
    setDiagonalMovement(false);
    // マンハッタン距離を採用
    setHeuristic(&Heuristic::manhattan);
    // 進める方向
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

// 斜め移動を設定
void AStar::Generator::setDiagonalMovement(bool enable_)
{
    // trueなら斜め移動を有効（8方向）
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    // heuristic_で計算
    // bindは関数をラップし関数オブジェクトをつくる（しかし今回はbind不要）
    heuristic = std::bind(heuristic_, _1, _2);
}

// 障害物を追加
void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

//　障害物を削除
void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    // シンプルな線形探索、指定された座標に一致するものがあれば削除
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

// 経路をA*で探索、座標リストを返す
AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    // 現在のノードをnullptrで初期化
    Node *current = nullptr;
    // Open（探索候補、未確定ノード）とClose（確定済みノード）の集合
    NodeSet openSet, closedSet;
    // メモリを確保
    openSet.reserve(100);
    closedSet.reserve(100);
    // 開始ノードをopenSetに入れる
    openSet.push_back(new Node(source_));

    // OpenSetが空になるまでループ
    while (!openSet.empty()) {
        // OpenSetの先頭にあるNodeのアドレスをcurrent_itに代入
        auto current_it = openSet.begin();
        // current_itが指すアドレスにあるNodeのアドレスをcurrentに代入
        current = *current_it;

        // openSetを線形走査して評価値が最小となるnodeとその対応イテレータ（アドレス）を保持
        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            // イテレータに対応するopenSetの要素（Node*）をnodeとする（つまりNodeポインタ型）
            auto node = *it;
            // node->getScore()は(*node).getScore()と同値 
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        // ターゲットに到達したらループを抜ける
        if (current->coordinates == target_) {
            break;
        }

        // currentをclosedSetに追加
        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            // 障害物に当たるかすでにclosedにnodeが存在する場合スキップ
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            // 新しいノードのコスト（G)を計算（直行移動なら10、斜め移動なら14）
            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            // openSetを線形走査して、newCoordinatesに一致するノードを返す
            Node *successor = findNodeOnList(openSet, newCoordinates);
            // successorがopenSetに存在しない場合は新しいノードを作成
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            // successorがopenSetに存在する場合は、totalCostが小さい場合は親ノードを更新
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    // 経路を復元
    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    // newしたNode*を解放（メモリリーク防止）
    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

// ノードを線形走査して、座標が一致するノードを返す
AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

// ノードを解放
void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

// 障害物との当たり判定。当たる時はtrueを返す
bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
