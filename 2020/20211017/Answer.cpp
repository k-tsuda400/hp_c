// HPC2020 公式サイト：
// https://www.hallab.co.jp/progcon/2020/

#include "Answer.hpp"
#include "Math.hpp"
#include <vector> // std::vectorに使用
#include <cassert> // アサートに使用
#include <iostream>

using namespace std;

//------------------------------------------------------------------------------
namespace hpc {

// ステージサイズは50x50で決め打ちなので、50x50で全マス探索
const int STAGE_WIDTH = 50;
const int STAGE_HEIGHT = 50;

// 次の行先を格納するリスト
vector<int> minCostNext;
int DecideNextTarget = 0;

// 山登り法を行うか
bool isYamanobori = false;
// 巻物数を格納する変数
int SCROLL_NUM = 0;

// デバッグ用
int STAGE_NUM = 0;

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
	// ウサギのスタート座標 + 各巻物の座標
	std::vector<int> relayX;
	std::vector<int> relayY;

	// まずステージ内の巻物数を数える
	// 数えつつ、各巻物の座標をvectorで保持する
	int scrNum = 0;

	// ベクトルの先頭にウサギのスタート座標を入れる
	relayX.push_back(aStage.rabbit().pos().x);
	relayY.push_back(aStage.rabbit().pos().y);

	for (auto scroll : aStage.scrolls()) {
		relayX.push_back(int(scroll.pos().x));
		relayY.push_back(int(scroll.pos().y));
		++scrNum;
	}
	// ウサギの分も追加
	scrNum += 1;
	SCROLL_NUM = scrNum;

	// pos単位のベクトルを得たいため、自前Vector2型のstd::Vectorを作成する
	std::vector<Vector2> relayList;
	std::vector<bool> relayList_isPassed;
	
	// その座標は既に通ったかを判定するbool配列 // <--- これ何でpushしてるの？

	// 2番目以降は巻物の座標
	for (auto i = 0; i < int(relayX.size()); ++i) {
		relayList.push_back( Vector2(float(relayX[i]), float(relayY[i])) );
		relayList_isPassed.push_back(false);
	}

	// 次に、各巻物間の「歩数コスト（≠ユークリッド距離）」をシミュレート
	// 2次元配列の参考：https://atcoder.jp/contests/APG4b/tasks/APG4b_t
	// 9999は無効な値（後で最小値調べるので一番でかい値入れてるだけ）
	vector<vector<int>> simuWalkNumList(SCROLL_NUM, vector<int>(SCROLL_NUM, 999999));

	for (int srcIdx = 0; srcIdx < SCROLL_NUM; ++srcIdx) {
		for (int tgtIdx = srcIdx; tgtIdx < SCROLL_NUM; ++tgtIdx) {
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
	// ここまで山登り下準備

	// ここから山登り

	// もし巻物が3つ以上あった場合、巻物を取る順番によって歩数が変わりそうなのでチェックフラグを立てる
	// -1 はウザギのスタート座標ぶん
	if (SCROLL_NUM - 1 > 2) {
		isYamanobori = true;
	}

	// debug: シミュレートうまくいってる？
	vector<int> debugGreedySimu;
	vector<int> debug2ndGreedySimu;


	// 普通の貪欲法の歩数をシミュレート
	if (isYamanobori) {
		int greedySimuWalk = 0;
		int nextTargetScr = 0; // 最初はウサギ座標へ
		int nextTargetScrKouho = 0;
		int INVALID_BIG_INT = 999999;
		relayList_isPassed.at(0) = true;
		for (auto i = 0; i < SCROLL_NUM; ++i) {
			int minWalkCost = INVALID_BIG_INT;
			for (auto jIdx = 0; jIdx < SCROLL_NUM; ++jIdx) {
				if (simuWalkNumList.at(nextTargetScr).at(jIdx) < minWalkCost && !relayList_isPassed.at(jIdx)) {
					minWalkCost = simuWalkNumList.at(nextTargetScr).at(jIdx);
					nextTargetScrKouho = jIdx;
				}
			}
			if (minWalkCost != INVALID_BIG_INT){
				nextTargetScr = nextTargetScrKouho;
				greedySimuWalk += minWalkCost;
				debugGreedySimu.push_back(minWalkCost);
				relayList_isPassed.at(nextTargetScr) = true;
			}			
		}
		for (auto i = 0; i < relayList_isPassed.size(); ++i) {
			//relayList_isPassedをfalseに初期化
			relayList_isPassed.at(i) = false;
		}


		// 最初に取る巻物をテキトーにしてみる
		// ここでは(巻物数+1)を2で割った値(小数点以下切り下げ)にしてる
		// 例：巻物数4 なら (4 + 1)/2 = 2.5 で小数点以下捨てて2
		int greedy2ndSimuWalk = 0;
		nextTargetScr = 0;
		relayList_isPassed.at(0) = true;
		for (auto i = 0; i < SCROLL_NUM; ++i) {
			int minWalkCost = INVALID_BIG_INT;
			/*
			if (i == 0) {
				nextTargetScr = int(SCROLL_NUM / 2.0);
				minWalkCost = simuWalkNumList.at(i).at(nextTargetScr);
				relayList_isPassed.at(nextTargetScr) = true;
			}
			*/
			{
				for (auto jIdx = 0; jIdx < SCROLL_NUM; ++jIdx) {
					if (simuWalkNumList.at(nextTargetScr).at(jIdx) < minWalkCost && !relayList_isPassed.at(jIdx)) {
						minWalkCost = simuWalkNumList.at(nextTargetScr).at(jIdx);
						nextTargetScrKouho = jIdx;
					}
				}
			}
			// 最初の取得巻物のみ、適当な巻物を選択
			if (minWalkCost != INVALID_BIG_INT && i == 0) {
				nextTargetScr = int(SCROLL_NUM / 2.0);
				greedy2ndSimuWalk += simuWalkNumList.at(0).at(nextTargetScr); // 最初はウサギ―。適当な巻物間
				debug2ndGreedySimu.push_back(greedy2ndSimuWalk);
				relayList_isPassed.at(nextTargetScr) = true;
			}
			
			if (minWalkCost != INVALID_BIG_INT && i != 0) {
				nextTargetScr = nextTargetScrKouho;
				greedy2ndSimuWalk += minWalkCost;
				debug2ndGreedySimu.push_back(minWalkCost);
				relayList_isPassed.at(nextTargetScr) = true;
			}
		}

		// 歩数が減ったらスタート地点を変えたほうを選択する
		if (greedySimuWalk <= greedy2ndSimuWalk) {
			isYamanobori = false;
		}

		int A = 0;
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

	float nearestDistanceToScr = 99999999.0f;
	int nearestDistanceToScrIdx = 0;

	int scrIdx = 0;

	int isGottenCount = 0;
	for (auto scroll : aStage.scrolls()) {
		// 最も近くかつ未取得の巻物を探す
		float distance = (scroll.pos().x - rabPos.x) * (scroll.pos().x - rabPos.x) +
			(scroll.pos().y - rabPos.y) * (scroll.pos().y - rabPos.y);
		if (nearestDistanceToScr > distance && !scroll.isGotten()) {
			nearestDistanceToScrIdx = scrIdx;
			nearestDistanceToScr = distance;
		}
		++scrIdx;

		if (scroll.isGotten()) {
			isGottenCount++;
		}
	}

	if (isYamanobori && isGottenCount == 0 && scrIdx != 1) {
		nearestDistanceToScrIdx = int(SCROLL_NUM / 2.0);
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
	STAGE_NUM++;
}

} // namespace
// EOF
