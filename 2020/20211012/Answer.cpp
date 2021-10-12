// HPC2020 公式サイト：
// https://www.hallab.co.jp/progcon/2020/

#include "Answer.hpp"
#include "Math.hpp"
#include <vector>
#include <cassert>

using namespace std;

//------------------------------------------------------------------------------
namespace hpc {

// ステージサイズは50x50で決め打ちなので、50x50で全マス探索
const int STAGE_WIDTH = 50;
const int STAGE_HEIGHT = 50;

// 次の行先を格納するリスト
vector<int> minCostNext;
int DecideNextTarget = 0;

//------------------------------------------------------------------------------
/// コンストラクタ
/// @detail 最初のステージ開始前に実行したい処理があればここに書きます
Answer::Answer()
{
}

//------------------------------------------------------------------------------
/// デストラクタ
/// @detail 最後のステージ終了後に実行したい処理があればここに書きます
Answer::~Answer()
{
}

//------------------------------------------------------------------------------
/// 各ステージ開始時に呼び出される処理
/// @detail 各ステージに対する初期化処理が必要ならここに書きます
/// @param aStage 現在のステージ
void Answer::initialize(const Stage& aStage)
{
	// ダイクストラ法を使うための中継点
	// (巻物の座標) + (巻物が存在しない中継点)
	std::vector<int> relayX;
	std::vector<int> relayY;

	// まずステージ内の巻物数を数える
	// 数えつつ、各巻物の座標をvectorで保持する
	int scrNum = 0;
	for (auto scroll : aStage.scrolls()) {
		relayX.push_back(int(scroll.pos().x));
		relayY.push_back(int(scroll.pos().y));
		++scrNum;
	}

	for (auto i = 0; i < STAGE_WIDTH - 1; ++i) {
		for (auto j = 0; j < STAGE_HEIGHT - 1; ++j) {
			auto checkedTerrainLeftUp = new Vector2(float(i), float(j));
			auto checkedTerrainLeftDown = new Vector2(float(i), float(j + 1));
			auto checkedTerrainRightUp = new Vector2(float(i + 1), float(j));
			auto checkedTerrainRightDown = new Vector2(float(i + 1), float(j + 1));

			// 4マス調べて、左上だけ水のパターン
			if (aStage.terrain(*checkedTerrainLeftUp) == hpc::Terrain::Pond &&
				aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Pond &&
				aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Pond &&
				aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Pond) {
				// 左上だけ水のパターンなら、右下を制御点とする
				relayX.push_back(i+1);
				relayY.push_back(j+1);
			}
		}
	}
	// pos単位のベクトルを得たいため、自前Vector2型のstd::Vectorを作成する
	std::vector<Vector2> relayList;
	for (auto i = 0; i < int(relayX.size()); ++i) {
		relayList.push_back( Vector2(float(relayX[i]), float(relayY[i])) );
	}

	// 次に、各巻物間の「歩数コスト（≠ユークリッド距離）」をシミュレート
	// 2次元配列の参考：https://atcoder.jp/contests/APG4b/tasks/APG4b_t
	vector<vector<int>> simuWalkNumList(int(relayX.size()), vector<int>(int(relayX.size()), -1));

	for (int srcIdx = 0; srcIdx < int(relayX.size()); ++srcIdx) {
		for (int tgtIdx = srcIdx; tgtIdx < int(relayX.size()); ++tgtIdx) {
			// 同じ座標だったら無視
			if (relayX[srcIdx] == relayX[tgtIdx] &&
				relayY[srcIdx] == relayY[tgtIdx]) {
				continue;
			}

			// まず、2点間の方向ベクトルを作成し、正規化
			float dirVecX = (relayX[tgtIdx] - relayX[srcIdx]) /
							Math::Sqrt((relayX[tgtIdx] - relayX[srcIdx]) * (relayX[tgtIdx] - relayX[srcIdx]) +
									   (relayY[tgtIdx] - relayY[srcIdx]) * (relayY[tgtIdx] - relayY[srcIdx]));

			float dirVecY = (relayY[tgtIdx] - relayY[srcIdx]) /
							Math::Sqrt((relayX[tgtIdx] - relayX[srcIdx]) * (relayX[tgtIdx] - relayX[srcIdx]) +
									   (relayY[tgtIdx] - relayY[srcIdx]) * (relayY[tgtIdx] - relayY[srcIdx]));

			// 次に、その巻物までの歩数をシミュレート
			// 下準備としてVector2型に変換
			Vector2 srcPos = Vector2(relayX[srcIdx], relayY[srcIdx]);
			Vector2 tgtPos = Vector2(relayX[tgtIdx], relayY[tgtIdx]);
			Vector2 dirVec = Vector2(dirVecX, dirVecY);
			// 歩数シミュレート関数にぶちこむ。
			// 引数：現在のスタート地点、目的地点、方向ベクトル、ステージ
			int simuWalkNum = calcSimuWalkNum(srcPos, tgtPos, dirVec, aStage);
			simuWalkNumList.at(srcIdx).at(tgtIdx) = simuWalkNum;
			simuWalkNumList.at(tgtIdx).at(srcIdx) = simuWalkNum;
		}
	}
	// ここまでダイクストラの下準備

	// ここからダイクストラ
	// そのノードに最も近いノードを決める（ただし一度も通っていない点に限る）
	vector<int> relayNext;
	vector<bool> isAlreadyPassedNext;

	for (auto i = 0; i < int(relayList.size()); ++i) {
		int minWalkCost = 999999;
		int minWalkCostIdx = 0;

		// ここで最もウサギに近い巻物を決める
		/*
		float minDistanceToRabbit = 999999.0f;
		int relayIdxMemory = 0;
		for (auto relayIdx = 0; relayIdx < int(relayList.size()); ++relayIdx) {
			float distance = (relayList.at(relayIdx).x - aStage.rabbit().pos().x) *
							(relayList.at(relayIdx).x - aStage.rabbit().pos().x) +
							(relayList.at(relayIdx).y - aStage.rabbit().pos().y) *
							(relayList.at(relayIdx).y - aStage.rabbit().pos().y);
			if (distance < minDistanceToRabbit) {
				minDistanceToRabbit = distance;
				relayIdxMemory = relayIdx;
			}
		}
		relayNext.push_back(relayIdxMemory);
		*/
		
		// 1つのノードに対して全ての中継点の距離を計算
		for (auto eachNodeIdx = 0; eachNodeIdx < int(relayList.size()); ++eachNodeIdx) {
			if (simuWalkNumList.at(i).at(eachNodeIdx) < minWalkCost && 
				!vector_finder(minCostNext, eachNodeIdx) &&
				i != eachNodeIdx) {
				minWalkCost = simuWalkNumList.at(i).at(eachNodeIdx);
				minWalkCostIdx = eachNodeIdx;
			}
		}
		// このノードにおける次の行先にする
		minCostNext.push_back(minWalkCostIdx);
	}
}

//------------------------------------------------------------------------------
/// 歩数をシミュレートする関数。引数を基に、歩数を返す。
/// 引数は全てconst。
int Answer::calcSimuWalkNum(const Vector2 srcVec, const Vector2 tgtVec, const Vector2 dirVec, const Stage& aStage)
{
	// 仮想ウサギ座標
	// (このタイミングで出発点をマスの中央にずらす)
	Vector2 simuRabPos = Vector2(srcVec.x + 0.5f, srcVec.y + 0.5f);
	// 仮想ターン数
	int simuWalkNum = 0;

	// 方向ベクトルの方向に応じて、4つのパターンを用意
	// =====
	// 1. x: tgtの方が大きい y: tgtの方が小さい（右上イメージ）
	if (dirVec.x >= 0 && dirVec.y <= 0)
	{
		bool isReached = false;

		// 仮想ウサギ座標がtarget座標に到達するまで、ループし続ける
		while (int(simuRabPos.x) < tgtVec.x &&
			   int(simuRabPos.y) > tgtVec.y ||
			   !isReached)
		{
			// 現在立っている地形を数値化
			assert((int)aStage.terrain(simuRabPos) > -1);
			float nowTerrainRate = convertTerrainToFloat(aStage.terrain(simuRabPos));

			simuRabPos.x += dirVec.x * nowTerrainRate;
			simuRabPos.y += dirVec.y * nowTerrainRate;
			++simuWalkNum;

			// ウサギがステージ内に収まっているか境界判定
			assert(simuRabPos.x > 0 && simuRabPos.x < STAGE_WIDTH);
			assert(simuRabPos.y > 0 && simuRabPos.y < STAGE_HEIGHT);

			if (int(simuRabPos.x) == int(tgtVec.x) &&
				int(simuRabPos.y) == int(tgtVec.y)) {
				isReached = true;
			}
		}
	}
	// =====
	// 2. x: tgtの方が大きい y: tgtの方が大きい（右下イメージ）
	else if (dirVec.x >= 0 && dirVec.y > 0)
	{
		bool isReached = false;

		// 仮想ウサギ座標がtarget座標に到達するまで、ループし続ける
		while (int(simuRabPos.x) < tgtVec.x &&
			   int(simuRabPos.y) < tgtVec.y ||
			   !isReached)
		{
			// 現在立っている地形を数値化
			assert((int)aStage.terrain(simuRabPos) > -1);
			float nowTerrainRate = convertTerrainToFloat(aStage.terrain(simuRabPos));

			simuRabPos.x += dirVec.x * nowTerrainRate;
			simuRabPos.y += dirVec.y * nowTerrainRate;
			++simuWalkNum;

			// ウサギがステージ内に収まっているか境界判定
			assert(simuRabPos.x > 0 && simuRabPos.x < STAGE_WIDTH);
			assert(simuRabPos.y > 0 && simuRabPos.y < STAGE_HEIGHT);

			if (int(simuRabPos.x) == int(tgtVec.x) &&
				int(simuRabPos.y) == int(tgtVec.y)) {
				isReached = true;
			}
		}
	}

	// =====
	// 3. x: tgtの方が小さい y: tgtの方が大きい（左下イメージ）
	else if (dirVec.x < 0 && dirVec.y > 0)
	{
		bool isReached = false;

		// 仮想ウサギ座標がtarget座標に到達するまで、ループし続ける
		while (int(simuRabPos.x) > tgtVec.x &&
			   int(simuRabPos.y) < tgtVec.y ||
			   !isReached)
		{
			// 現在立っている地形を数値化
			assert((int)aStage.terrain(simuRabPos) > -1);
			float nowTerrainRate = convertTerrainToFloat(aStage.terrain(simuRabPos));

			simuRabPos.x += dirVec.x * nowTerrainRate;
			simuRabPos.y += dirVec.y * nowTerrainRate;
			++simuWalkNum;

			// ウサギがステージ内に収まっているか境界判定
			assert(simuRabPos.x > 0 && simuRabPos.x < STAGE_WIDTH);
			assert(simuRabPos.y > 0 && simuRabPos.y < STAGE_HEIGHT);

			if (int(simuRabPos.x) == int(tgtVec.x) &&
				int(simuRabPos.y) == int(tgtVec.y)) {
				isReached = true;
			}
		}
	}

	// =====
	// 4. x: tgtの方が小さい y: tgtの方が小さい（左上イメージ）
	else if (dirVec.x < 0 && dirVec.y <= 0)
	{
		bool isReached = false;

		// 仮想ウサギ座標がtarget座標に到達するまで、ループし続ける
		while (int(simuRabPos.x) > tgtVec.x &&
			   int(simuRabPos.y) > tgtVec.y ||
			   !isReached)
		{
			// 現在立っている地形を数値化
			assert((int)aStage.terrain(simuRabPos) > -1);
			float nowTerrainRate = convertTerrainToFloat(aStage.terrain(simuRabPos));

			simuRabPos.x += dirVec.x * nowTerrainRate;
			simuRabPos.y += dirVec.y * nowTerrainRate;
			++simuWalkNum;

			// ウサギがステージ内に収まっているか境界判定
			assert(simuRabPos.x > 0 && simuRabPos.x < STAGE_WIDTH);
			assert(simuRabPos.y > 0 && simuRabPos.y < STAGE_HEIGHT);

			if (int(simuRabPos.x) == int(tgtVec.x) &&
				int(simuRabPos.y) == int(tgtVec.y)) {
				isReached = true;
			}
		}
	}

	assert(simuWalkNum > 0);
	return simuWalkNum;
}

//------------------------------------------------------------------------------
/// 与えられたterrainのジャンプ力比率をfloatとして返す関数
float Answer::convertTerrainToFloat(const Terrain terrain)
{
	// 現在立っている地形を数値化
	float nowTerrainRate = 0.0f;

	switch (terrain) {
		case Terrain::Plain:
			nowTerrainRate = 1.0f;
			break;
		case Terrain::Bush:
			nowTerrainRate = 0.6f;
			break;
		case Terrain::Sand:
			nowTerrainRate = 0.3f;
			break;
		case Terrain::Pond:
			nowTerrainRate = 0.1f;
			break;
		default:
			// 地形情報が不正
			assert(nowTerrainRate > 0.0f);
			break;
	}

	return nowTerrainRate;
}

//------------------------------------------------------------------------------
/// std::vector内に指定した数字numberがあるかを判定する関数
/// 参考：https://qiita.com/wonder_zone/items/51fb5c3a773b98aa130c
bool Answer::vector_finder(std::vector<int> vec, int number) {
	auto itr = std::find(vec.begin(), vec.end(), number);
	size_t index = std::distance(vec.begin(), itr);
	if (index != vec.size()) { // 発見できたとき
		return true;
	}
	else { // 発見できなかったとき
		return false;
	}
}

//------------------------------------------------------------------------------
/// 毎フレーム呼び出される処理
/// @detail 移動先を決定して返します
/// @param aStage 現在のステージ
/// @return 移動の目標座標
Vector2 Answer::getTargetPos(const Stage& aStage)
{
    auto rabPos = aStage.rabbit().pos();

	float nearestDistanceToScr = 99999999.0;
	int nearestDistanceToScrIdx = 0;

	int scrIdx = 0;
	for (auto scroll : aStage.scrolls()) {
		// 最も近くかつ未取得の巻物を探す
		float distance = (scroll.pos().x - rabPos.x) * (scroll.pos().x - rabPos.x) +
						 (scroll.pos().y - rabPos.y) * (scroll.pos().y - rabPos.y);
		if (nearestDistanceToScr > distance && !scroll.isGotten()) {
			nearestDistanceToScrIdx = scrIdx;
			nearestDistanceToScr = distance;
		}
		++scrIdx;
	}

	scrIdx = 0;
    for(auto scroll : aStage.scrolls()) {

        // 最も近くかつ未取得の巻物に向かって飛ぶ
		if (scrIdx == nearestDistanceToScrIdx && !scroll.isGotten()){
            return scroll.pos();
        }

		scrIdx = scrIdx + 1;
    }

    return rabPos;
}

//------------------------------------------------------------------------------
/// 各ステージ終了時に呼び出される処理
/// @detail 各ステージに対する終了処理が必要ならここに書きます
/// @param aStage 現在のステージ
void Answer::finalize(const Stage& aStage)
{
}

} // namespace
// EOF
