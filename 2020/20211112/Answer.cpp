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
	// 目的点にどれくらい寄るか。
	// 0.0が最小（全てのマスで左上を基準に）
	// 0.9999... が最大
	const float CORNER_NEARBY_RATIO = 0.75f;
	const float CORNER_CENTER_RATIO = 0.5f;

	// 次の行先を格納するリスト
	vector<int> minCostNext;
	int DecideNextTarget = 0;

	// どの順番で巻物を取るか、シミュレーション後に「取る順Index」を格納するリスト
	vector<int> orderGetScr;
	vector<int> orderGetScr2nd;
	vector<int> orderGetScr2ndTmp;

	// 山登り法を行うか
	bool isYamanobori = false;
	// 巻物数を格納する変数
	int SCROLL_NUM = 0;

	// デバッグ用
	int STAGE_NUM = 0;
	int debugTurnNum = 0;
	vector<int> debugGreedySimu;
	vector<int> debug2ndGreedySimu;

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
		// 中継点リストの初期化
		_RelayX.clear();
		_RelayY.clear();
		_RelayList.clear();

		// まずステージ内の巻物数を数える
		// 数えつつ、各巻物の座標をvectorで保持する
		int scrNum = 0;

		// ベクトルの先頭にウサギのスタート座標を入れる
		_RelayX.push_back(aStage.rabbit().pos().x);
		_RelayY.push_back(aStage.rabbit().pos().y);

		for (auto scroll : aStage.scrolls()) {
			_RelayX.push_back(scroll.pos().x);
			_RelayY.push_back(scroll.pos().y);
			++scrNum;
		}
		// ウサギの分も追加
		scrNum += 1;
		SCROLL_NUM = scrNum;

		// pos単位のベクトルを得たいため、自前Vector2型のstd::Vectorを作成する
		//std::vector<Vector2> mRelayList; // <-- これはメンバ変数に昇格した
		std::vector<bool> relayList_isPassed;

		// その座標は既に通ったかを判定するbool配列 // <--- これ何でpushしてるの？

		// 2番目以降は巻物の座標
		for (auto i = 0; i < int(_RelayX.size()); ++i) {
			_RelayList.push_back(Vector2(_RelayX[i], _RelayY[i]));
			relayList_isPassed.push_back(false);
		}

		// 既にrelayListに中継点を追加して、歩数のコスト計算も行うが、
		// 巻物を取る順番には関与しない
		for (auto i = 0; i < STAGE_WIDTH - 1; ++i) {
			for (auto j = 0; j < STAGE_HEIGHT - 1; ++j) {
				auto checkedTerrainLeftUp = new Vector2(float(i), float(j));
				auto checkedTerrainLeftDown = new Vector2(float(i), float(j + 1));
				auto checkedTerrainRightUp = new Vector2(float(i + 1), float(j));
				auto checkedTerrainRightDown = new Vector2(float(i + 1), float(j + 1));
				
				// 土地情報だけで決められる中継点
				addRelayPointOfPondCorner(aStage, i, j);
				addRelayPointOfPondCross2_2(aStage, i, j);
				addRelayPointOfSandCorner(aStage, i, j);
				addRelayPointOfPondPlain3_1(aStage, i, j);
			}
		}

		// 巻物が水中にあり、その近くが土地の場合はその地点も中継点としたい
		addRelayPointInPondScroll(aStage);

		// 次に、各巻物間の「歩数コスト（≠ユークリッド距離）」をシミュレート
		// 2次元配列の参考：https://atcoder.jp/contests/APG4b/tasks/APG4b_t
		// 9999は無効な値（後で最小値調べるので一番でかい値入れてるだけ）
		//vector<vector<int>> simuWalkNumList(SCROLL_NUM, vector<int>(SCROLL_NUM, 999999));
		vector<vector<vector<int>>> simuWalkNumList(_RelayList.size(), vector<vector<int>>(_RelayList.size(), vector<int>(SCROLL_NUM, 999999)));

		for (int srcIdx = 0; srcIdx < _RelayList.size() ; ++srcIdx) {
			for (int tgtIdx = srcIdx; tgtIdx < _RelayList.size() ; ++tgtIdx) {
				// 取得巻物数ごとに変化
				for (int getScrNumIdx = 0; getScrNumIdx < SCROLL_NUM; ++getScrNumIdx) {

					// 同じ座標だったら無視
					if (int(_RelayX[srcIdx]) == int(_RelayX[tgtIdx]) &&
						int(_RelayY[srcIdx]) == int(_RelayY[tgtIdx])) {
						continue;
					}

					// まず、2点間の方向ベクトルを作成し、正規化
					float dirVecX = (_RelayX[tgtIdx] - _RelayX[srcIdx]) /
						Math::Sqrt((_RelayX[tgtIdx] - _RelayX[srcIdx]) * (_RelayX[tgtIdx] - _RelayX[srcIdx]) +
						(_RelayY[tgtIdx] - _RelayY[srcIdx]) * (_RelayY[tgtIdx] - _RelayY[srcIdx]));

					float dirVecY = (_RelayY[tgtIdx] - _RelayY[srcIdx]) /
						Math::Sqrt((_RelayX[tgtIdx] - _RelayX[srcIdx]) * (_RelayX[tgtIdx] - _RelayX[srcIdx]) +
						(_RelayY[tgtIdx] - _RelayY[srcIdx]) * (_RelayY[tgtIdx] - _RelayY[srcIdx]));

					// 次に、その巻物までの歩数をシミュレート
					// 下準備としてVector2型に変換
					Vector2 srcPos = Vector2(_RelayX[srcIdx], _RelayY[srcIdx]);
					Vector2 tgtPos = Vector2(_RelayX[tgtIdx], _RelayY[tgtIdx]);
					Vector2 dirVec = Vector2(dirVecX, dirVecY);
					// 歩数シミュレート関数にぶちこむ。
					// 引数：現在のスタート地点、目的地点、方向ベクトル、ステージ
					int simuWalkNum = calcSimuWalkNum(srcPos, tgtPos, dirVec, getScrNumIdx, aStage);
					simuWalkNumList.at(srcIdx).at(tgtIdx).at(getScrNumIdx) = simuWalkNum;
					simuWalkNumList.at(tgtIdx).at(srcIdx).at(getScrNumIdx) = simuWalkNum;

				}
			}
		}

		isYamanobori = true;

		// 普通の貪欲法の歩数をシミュレート
		int greedySimuWalk = 0;
		int nextTargetScr = 0; // 最初はウサギ座標へ
		int nextTargetScrKouho = 0;
		int INVALID_BIG_INT = 999999;
		relayList_isPassed.at(0) = true;
		for (auto i = 0; i < SCROLL_NUM; ++i) {
			int minWalkCost = INVALID_BIG_INT;
			for (auto jIdx = 0; jIdx < SCROLL_NUM; ++jIdx) {
				if (simuWalkNumList.at(nextTargetScr).at(jIdx).at(i) < minWalkCost && !relayList_isPassed.at(jIdx)) {
					minWalkCost = simuWalkNumList.at(nextTargetScr).at(jIdx).at(i);
					nextTargetScrKouho = jIdx;
				}
			}
			if (minWalkCost != INVALID_BIG_INT) {
				nextTargetScr = nextTargetScrKouho;
				greedySimuWalk += minWalkCost;
				debugGreedySimu.push_back(minWalkCost);
				orderGetScr.push_back(nextTargetScr - 1);
				relayList_isPassed.at(nextTargetScr) = true;
			}
		}

		vector<int> debug_yamanobori;

		// 最初をテキトーにする
		int greedySimuWalk2nd = 999999;
		int greedySimuWalk2ndTmp = 0;
		int startScrDecide = 0;
		for (int startScr = 1; startScr < SCROLL_NUM; ++startScr) {
			nextTargetScr = 0; // 最初はウサギ座標へ
			nextTargetScrKouho = 0;
			greedySimuWalk2ndTmp = 0;

			for (auto i = 0; i < relayList_isPassed.size(); ++i) {
				//relayList_isPassedをfalseに初期化
				relayList_isPassed.at(i) = false;
			}
			relayList_isPassed.at(0) = true;

			for (auto i = 0; i < SCROLL_NUM; ++i) {
				int minWalkCost = INVALID_BIG_INT;
				
				
				// 最初の1ターン
				if (i == 0) {
					int tekitoNum = startScr;
					minWalkCost = simuWalkNumList.at(nextTargetScr).at(tekitoNum).at(i);
					nextTargetScrKouho = tekitoNum;
				}
				else {
					for (auto jIdx = 0; jIdx < SCROLL_NUM; ++jIdx) {
						if (simuWalkNumList.at(nextTargetScr).at(jIdx).at(i) < minWalkCost && !relayList_isPassed.at(jIdx)) {
							minWalkCost = simuWalkNumList.at(nextTargetScr).at(jIdx).at(i);
							nextTargetScrKouho = jIdx;
						}
					}
				}

				if (minWalkCost != INVALID_BIG_INT) {
					nextTargetScr = nextTargetScrKouho;
					greedySimuWalk2ndTmp += minWalkCost;
					debug2ndGreedySimu.push_back(minWalkCost);
					orderGetScr2ndTmp.push_back(nextTargetScr - 1);
					relayList_isPassed.at(nextTargetScr) = true;
				}
			}
			debug2ndGreedySimu.clear();
			debug_yamanobori.push_back(greedySimuWalk2ndTmp);

			if (greedySimuWalk2ndTmp < greedySimuWalk2nd) {
				startScrDecide = startScr;
				greedySimuWalk2nd = greedySimuWalk2ndTmp;
				orderGetScr2nd.clear();
				for (auto cpyIdx = 0; cpyIdx < SCROLL_NUM-1; ++cpyIdx) {
					orderGetScr2nd.push_back(orderGetScr2ndTmp[cpyIdx]);
				}
			}
			orderGetScr2ndTmp.clear();
		}		

		// 歩数が減ったらスタート地点を変えたほうを選択する
		if (greedySimuWalk <= greedySimuWalk2nd) {
			isYamanobori = false;
		}

		// --------------------------------------------------
		// ここまではあくまで巻物を取る順番を決めただけ
		// 次に、歩数コストを計算する点に「中継点」を追加
		
		
		// そして、巻物取る順の配列に対して中継点の情報を追加した新しい「ウサギの動く順配列」を新規で作る
		

		for (auto startScr = 0; startScr < SCROLL_NUM-1; ++startScr) {
			isYamanobori ? calcRelayFromDijkstra(startScr, orderGetScr2nd, simuWalkNumList) :
						   calcRelayFromDijkstra(startScr, orderGetScr, simuWalkNumList);
		}

		

		// もし目的地が1個しか無い場合、中継点を再帰で増やす処理は不適切なのでスキップする
		if (_OrderGetScrWithRelay.size() == 1) {
			// 何もしない
		}
		else {
			// 関数を再帰的に実行してみる
			vector<int> tmpVec; // <---  1回目の中継点追加結果を反映した配列をコピーする用の配列
			for (int iTmp = 0; iTmp < _OrderGetScrWithRelay.size(); ++iTmp) {
				tmpVec.push_back(_OrderGetScrWithRelay.at(iTmp));
			}

			_DijkstraGetScrNum = 0;
			_OrderGetScrWithRelay.clear();
			for (auto startScr = 0; startScr < tmpVec.size(); ++startScr) {
				calcRelayFromDijkstra(startScr, tmpVec, simuWalkNumList);
			}

			// 初期値は10だったが、時間かかりすぎるので減らしたい
			for (int recursiveCnt = 0; recursiveCnt < 10; ++recursiveCnt) {
				// 関数を再帰的に実行してみる
				tmpVec.clear();
				for (int iTmp = 0; iTmp < _OrderGetScrWithRelay.size(); ++iTmp) {
					tmpVec.push_back(_OrderGetScrWithRelay.at(iTmp));
				}

				_DijkstraGetScrNum = 0;
				_OrderGetScrWithRelay.clear();
				for (auto startScr = 0; startScr < tmpVec.size(); ++startScr) {
					calcRelayFromDijkstra(startScr, tmpVec, simuWalkNumList);
				}
			}
		}
	}

	// 中継点を作成する関数群

	// 水辺の角4パターンを制御点とする---------------------------------
	// oo (oは陸面、xは水面としたときのイメージ図)
	// xo
	void Answer::addRelayPointOfPondCorner(const Stage& aStage, int aX, int aY)
	{
		auto checkedTerrainLeftUp = new Vector2(float(aX), float(aY));
		auto checkedTerrainLeftDown = new Vector2(float(aX), float(aY + 1));
		auto checkedTerrainRightUp = new Vector2(float(aX + 1), float(aY));
		auto checkedTerrainRightDown = new Vector2(float(aX + 1), float(aY + 1));

		// １．4マス調べて、左上だけ水のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Pond) {
			// 左上だけ水のパターンなら、右下を制御点とする
			_RelayX.push_back(float(aX + 1) + (1 - CORNER_NEARBY_RATIO));
			_RelayY.push_back(float(aY + 1) + (1 - CORNER_NEARBY_RATIO));
			_RelayList.push_back(Vector2(float(aX + 1) + (1 - CORNER_NEARBY_RATIO),
										 float(aY + 1) + (1 - CORNER_NEARBY_RATIO)));
		}
		// ２．4マス調べて、右上だけ水のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Pond) {
			// 右上だけ水のパターンなら、左下を制御点とする
			_RelayX.push_back(float(aX) + CORNER_NEARBY_RATIO);
			_RelayY.push_back(float(aY + 1) + (1 - CORNER_NEARBY_RATIO));
			_RelayList.push_back(Vector2(float(aX) + CORNER_NEARBY_RATIO, 
										 float(aY + 1) + (1 - CORNER_NEARBY_RATIO)));
		}
		// ３．4マス調べて、左下だけ水のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Pond) {
			// 左下だけ水のパターンなら、右上を制御点とする
			_RelayX.push_back(float(aX + 1) + (1 - CORNER_NEARBY_RATIO));
			_RelayY.push_back(float(aY) + CORNER_NEARBY_RATIO);
			_RelayList.push_back(Vector2(float(aX + 1) + (1 - CORNER_NEARBY_RATIO),
										 float(aY) + CORNER_NEARBY_RATIO));
		}
		// ４．4マス調べて、右下だけ水のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) == hpc::Terrain::Pond) {
			// 右下だけ水のパターンなら、左上を制御点とする
			_RelayX.push_back(float(aX) + CORNER_NEARBY_RATIO);
			_RelayY.push_back(float(aY) + CORNER_NEARBY_RATIO);
			_RelayList.push_back(Vector2(float(aX) + CORNER_NEARBY_RATIO, 
										 float(aY) + CORNER_NEARBY_RATIO));
		}
	}

	// 水辺がクロスするパターンを制御点とする---------------------------------
	// ox (oは陸面、xは水面としたときのイメージ図)
	// xo
	void Answer::addRelayPointOfPondCross2_2(const Stage& aStage, int aX, int aY)
	{
		auto checkedTerrainLeftUp = new Vector2(float(aX), float(aY));
		auto checkedTerrainLeftDown = new Vector2(float(aX), float(aY + 1));
		auto checkedTerrainRightUp = new Vector2(float(aX + 1), float(aY));
		auto checkedTerrainRightDown = new Vector2(float(aX + 1), float(aY + 1));

		// １．左上・右下が平面のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Pond) {
			// 4マスのクロス中心を制御点とする
			_RelayX.push_back(float(aX + 1));
			_RelayY.push_back(float(aY + 1));
			_RelayList.push_back(Vector2(float(aX + 1), float(aY + 1)));
		}
		// ２．左下・右上が平面のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) == hpc::Terrain::Pond) {
			// 4マスのクロス中心を制御点とする
			_RelayX.push_back(float(aX + 1));
			_RelayY.push_back(float(aY + 1));
			_RelayList.push_back(Vector2(float(aX + 1), float(aY + 1)));
		}
	}

	// 砂の角4パターンを制御点とする---------------------------------
	// oo (oは平地・草、vは砂場としたときのイメージ図)
	// vo
	void Answer::addRelayPointOfSandCorner(const Stage& aStage, int aX, int aY)
	{
		auto checkedTerrainLeftUp = new Vector2(float(aX), float(aY));
		auto checkedTerrainLeftDown = new Vector2(float(aX), float(aY + 1));
		auto checkedTerrainRightUp = new Vector2(float(aX + 1), float(aY));
		auto checkedTerrainRightDown = new Vector2(float(aX + 1), float(aY + 1));

		// １．4マス調べて、左上だけ砂のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) == hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Sand) {
			// 左上だけ砂のパターンなら、右下を制御点とする
			// （ただし右下が水なら制御点としない）
			if (aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Pond) {
				_RelayX.push_back(float(aX + 1) + (1 - CORNER_NEARBY_RATIO));
				_RelayY.push_back(float(aY + 1) + (1 - CORNER_NEARBY_RATIO));
				_RelayList.push_back(Vector2(float(aX + 1) + (1 - CORNER_NEARBY_RATIO),
											 float(aY + 1) + (1 - CORNER_NEARBY_RATIO)));
			}
		}
		// ２．4マス調べて、右上だけ砂のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainRightUp) == hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Sand) {
			// 右上だけ砂のパターンなら、左下を制御点とする
			// （ただし左下が水なら制御点としない）
			if (aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Pond) {
				_RelayX.push_back(float(aX) + CORNER_NEARBY_RATIO);
				_RelayY.push_back(float(aY + 1) + (1 - CORNER_NEARBY_RATIO));
				_RelayList.push_back(Vector2(float(aX) + CORNER_NEARBY_RATIO, 
											 float(aY + 1) + (1 - CORNER_NEARBY_RATIO)));
			}
		}
		// ３．4マス調べて、左下だけ砂のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainLeftDown) == hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Sand) {
			// 左下だけ砂のパターンなら、右上を制御点とする
			// （ただし右上が水なら制御点としない）
			if (aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Pond) {
				_RelayX.push_back(float(aX + 1) + (1 - CORNER_NEARBY_RATIO));
				_RelayY.push_back(float(aY) + CORNER_NEARBY_RATIO);
				_RelayList.push_back(Vector2(float(aX + 1) + (1 - CORNER_NEARBY_RATIO),
											 float(aY) + CORNER_NEARBY_RATIO));
			}
		}
		// ４．4マス調べて、右下だけ砂のパターン
		if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Sand &&
			aStage.terrain(*checkedTerrainRightDown) == hpc::Terrain::Sand) {
			// 左下だけ砂のパターンなら、右上を制御点とする
			// （ただし右上が水なら制御点としない）
			if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Pond) {
				_RelayX.push_back(float(aX) + CORNER_NEARBY_RATIO);
				_RelayY.push_back(float(aY) + CORNER_NEARBY_RATIO);
				_RelayList.push_back(Vector2(float(aX) + CORNER_NEARBY_RATIO, 
											 float(aY) + CORNER_NEARBY_RATIO));
			}
		}
	}

	// ほぼ水辺の4マスなら、残り1マスの非水マスを制御点とする---------------------------------
	// xo (oは陸面、xは水面としたときのイメージ図)
	// xx
	void Answer::addRelayPointOfPondPlain3_1(const Stage& aStage, int aX, int aY)
	{
		auto checkedTerrainLeftUp = new Vector2(float(aX), float(aY));
		auto checkedTerrainLeftDown = new Vector2(float(aX), float(aY + 1));
		auto checkedTerrainRightUp = new Vector2(float(aX + 1), float(aY));
		auto checkedTerrainRightDown = new Vector2(float(aX + 1), float(aY + 1));

		// １．4マス調べて、左上だけ水じゃないパターン
		if (aStage.terrain(*checkedTerrainLeftUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) == hpc::Terrain::Pond) {
			_RelayX.push_back(float(aX) + CORNER_CENTER_RATIO);
			_RelayY.push_back(float(aY) + CORNER_CENTER_RATIO);
			_RelayList.push_back(Vector2(float(aX) + CORNER_CENTER_RATIO, 
										 float(aY) + CORNER_CENTER_RATIO));
		}
		// ２．4マス調べて、右上だけ水じゃないパターン
		if (aStage.terrain(*checkedTerrainLeftUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) == hpc::Terrain::Pond) {
			_RelayX.push_back(float(aX + 1) + CORNER_CENTER_RATIO);
			_RelayY.push_back(float(aY) + CORNER_CENTER_RATIO);
			_RelayList.push_back(Vector2(float(aX + 1) + CORNER_CENTER_RATIO, 
										 float(aY) + CORNER_CENTER_RATIO));
		}
		// ３．4マス調べて、左下だけ水じゃないパターン
		if (aStage.terrain(*checkedTerrainLeftUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) != hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) == hpc::Terrain::Pond) {
			_RelayX.push_back(float(aX) + CORNER_CENTER_RATIO);
			_RelayY.push_back(float(aY + 1) + CORNER_CENTER_RATIO);
			_RelayList.push_back(Vector2(float(aX) + CORNER_CENTER_RATIO, 
										 float(aY + 1) + CORNER_CENTER_RATIO));
		}
		// ４．4マス調べて、右下だけ水じゃないパターン
		if (aStage.terrain(*checkedTerrainLeftUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainLeftDown) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightUp) == hpc::Terrain::Pond &&
			aStage.terrain(*checkedTerrainRightDown) != hpc::Terrain::Pond) {
			_RelayX.push_back(float(aX + 1) + CORNER_CENTER_RATIO);
			_RelayY.push_back(float(aY + 1) + CORNER_CENTER_RATIO);
			_RelayList.push_back(Vector2(float(aX + 1) + CORNER_CENTER_RATIO, 
										 float(aY + 1) + CORNER_CENTER_RATIO));
		}
	}

	// 水中にある巻物の上下左右の延長線上の陸の境界に中継点を置く
	void Answer::addRelayPointInPondScroll(const Stage& aStage)
	{
		for (auto scroll : aStage.scrolls()) {
			// もし巻物が水中ではなかったらスルー
			if (aStage.terrain(scroll.pos()) != hpc::Terrain::Pond) {
				continue;
			}

			// まず縦に調査
			// １．巻物の上方向の水/陸の境界を探る
			Vector2 ResearchTerrain = scroll.pos();
			while (1) {
				// 1つ上のグリッドへ移動
				ResearchTerrain.y -= 1.0f;

				// ステージの外へ行ったら調査終了。中継点は追加しない
				if (ResearchTerrain.y < 0.0f) {
					break;
				}
				
				if (aStage.terrain(ResearchTerrain) != hpc::Terrain::Pond) {
					// 水/陸の境界を中継点として追加					
					_RelayX.push_back(ResearchTerrain.x);
					_RelayY.push_back(ResearchTerrain.y);
					_RelayList.push_back(Vector2(ResearchTerrain.x, ResearchTerrain.y));
					break;
				}
			}
			// ２．巻物の下方向の水/陸の境界を探る
			ResearchTerrain = scroll.pos();
			while (1) {
				// 1つ下のグリッドへ移動
				ResearchTerrain.y += 1.0f;

				// ステージの外へ行ったら調査終了。中継点は追加しない
				if (ResearchTerrain.y > float(STAGE_HEIGHT)) {
					break;
				}

				if (aStage.terrain(ResearchTerrain) != hpc::Terrain::Pond) {
					// 水/陸の境界を中継点として追加					
					_RelayX.push_back(ResearchTerrain.x);
					_RelayY.push_back(ResearchTerrain.y);
					_RelayList.push_back(Vector2(ResearchTerrain.x, ResearchTerrain.y));
					break;
				}
			}

			// 次に横に調査
			// ３．巻物の左方向の水/陸の境界を探る
			ResearchTerrain = scroll.pos();
			while (1) {
				// 1つ上のグリッドへ移動
				ResearchTerrain.x -= 1.0f;

				// ステージの外へ行ったら調査終了。中継点は追加しない
				if (ResearchTerrain.x < 0.0f) {
					break;
				}

				if (aStage.terrain(ResearchTerrain) != hpc::Terrain::Pond) {
					// 水/陸の境界を中継点として追加					
					_RelayX.push_back(ResearchTerrain.x);
					_RelayY.push_back(ResearchTerrain.y);
					_RelayList.push_back(Vector2(ResearchTerrain.x, ResearchTerrain.y));
					break;
				}
			}

			// ４．巻物の右方向の水/陸の境界を探る
			ResearchTerrain = scroll.pos();
			while (1) {
				// 1つ上のグリッドへ移動
				ResearchTerrain.x += 1.0f;

				// ステージの外へ行ったら調査終了。中継点は追加しない
				if (ResearchTerrain.x > float(STAGE_WIDTH)) {
					break;
				}

				if (aStage.terrain(ResearchTerrain) != hpc::Terrain::Pond) {
					// 水/陸の境界を中継点として追加					
					_RelayX.push_back(ResearchTerrain.x);
					_RelayY.push_back(ResearchTerrain.y);
					_RelayList.push_back(Vector2(ResearchTerrain.x, ResearchTerrain.y));
					break;
				}
			}
		}
	}

	//------------------------------------------------------------------------------
	// ある2つの巻物間でダイクストラ法を実行し、巻物を取る順を格納した配列に中継点の情報を追加していく関数
	// 関数が受け取るのは
	//・「スタート巻物ノードの値（整数）(不変なのでconst渡し)」
	//・「巻物取る順の配列(不変なのでconst渡し)」
	void Answer::calcRelayFromDijkstra(const int startScrNum, const std::vector<int> vec, const vector<vector<vector<int>>> walkCostList)
	{
		// 入力のstartScrNumは目標の巻物
		// なので、例えば目標巻物が「0」だけなら、
		//「(中継点1)、0」みたいな_OrderGetScrWithRealy配列を作成したい。

		// まず、startScrNumがゼロか否か判定：
		// ゼロ　→　スタートのウサギノード(index:0)からvec[0]の巻物ノード間でダイクストラ
		// 非ゼロ→　vec[startScrNum-1]の巻物ノードからvec[startScrNum]の巻物ノード間でダイクストラ

		// ただ正直、ダイクストラ法の実装法がわからなかたっため、
		// 今回はあくまで「中継点を1つ挟んだ時に歩数が改善されるか」

		int addRelayIndex = 0;
		int defaultWalkCost;
		int minWalkCost;

		if (startScrNum == 0) {
			defaultWalkCost = walkCostList.at(0).at(vec.at(startScrNum) + 1).at(_DijkstraGetScrNum);
			minWalkCost = defaultWalkCost;
			for (int i = 0; i < _RelayList.size(); ++i) {
				int relayWalkCost = walkCostList.at(0).at(i).at(_DijkstraGetScrNum) +
					walkCostList.at(i).at(vec.at(startScrNum) + 1).at(_DijkstraGetScrNum);
				if (minWalkCost > relayWalkCost) {
					addRelayIndex = i - 1;
					minWalkCost = relayWalkCost;
				}
			}
		}
		else {
			defaultWalkCost = walkCostList.at(vec.at(startScrNum-1) + 1).at(vec.at(startScrNum) + 1).at(_DijkstraGetScrNum);
			minWalkCost = defaultWalkCost;
			for (int i = 0; i < _RelayList.size(); ++i) {
				int relayWalkCost = walkCostList.at(vec.at(startScrNum - 1) + 1).at(i).at(_DijkstraGetScrNum) +
					walkCostList.at(i).at(vec.at(startScrNum) + 1).at(_DijkstraGetScrNum);
				if (minWalkCost > relayWalkCost) {
					addRelayIndex = i - 1;
					minWalkCost = relayWalkCost;
				}
			}
		}		

		// もし中継点を経由した方が歩数が少なくなる場合は、中継点を目的地に追加する
		if (minWalkCost != defaultWalkCost) {
			_OrderGetScrWithRelay.push_back(addRelayIndex);
		}
		_OrderGetScrWithRelay.push_back(vec[startScrNum]);

		// もし目的地が巻物ならば、取得巻物数を+1する
		if (vec[startScrNum] <= SCROLL_NUM-1) {
			++_DijkstraGetScrNum;
		}
		// SCROLL_NUM - 1が「ステージ内の巻物の数」なので、_Dijkstra変数は(SCROLL_NUM - 1)まで増やしてOK
		// それ以上はout_of_rangeエラー
		if (_DijkstraGetScrNum > SCROLL_NUM - 1) {
			_DijkstraGetScrNum = SCROLL_NUM - 1;
		}
	}

	//------------------------------------------------------------------------------
	/// 歩数をシミュレートする関数。引数を基に、歩数を返す。
	/// 引数は全てconst。
	int Answer::calcSimuWalkNum(const Vector2 srcVec, const Vector2 tgtVec, const Vector2 dirVec, const int getScrNum, const Stage& aStage)
	{
		// 仮想ウサギ座標
		// (このタイミングで出発点をマスの中央にずらす)
		Vector2 simuRabPos = Vector2(srcVec.x, srcVec.y);
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

				simuRabPos.x += dirVec.x * nowTerrainRate * pow(1.1f, getScrNum);
				simuRabPos.y += dirVec.y * nowTerrainRate * pow(1.1f, getScrNum);
				++simuWalkNum;

				// ウサギがステージ内に収まっているか境界判定
				//assert(simuRabPos.x > 0 && simuRabPos.x < STAGE_WIDTH);
				//assert(simuRabPos.y > 0 && simuRabPos.y < STAGE_HEIGHT);
				if (simuRabPos.x < 0) simuRabPos.x = 0.0f;
				if (simuRabPos.x > STAGE_WIDTH) simuRabPos.x = float(STAGE_HEIGHT - 1);
				if (simuRabPos.y < 0) simuRabPos.y = 0.0f;
				if (simuRabPos.y > STAGE_HEIGHT) simuRabPos.y = float(STAGE_HEIGHT - 1);

				if (int(simuRabPos.x) >= int(tgtVec.x) &&
					int(simuRabPos.y) <= int(tgtVec.y)) {
					isReached = true;
				}

				// 計算は200ターンで打ち切り
				if (simuWalkNum > 200) {
					break;
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

				simuRabPos.x += dirVec.x * nowTerrainRate * pow(1.1f, getScrNum);
				simuRabPos.y += dirVec.y * nowTerrainRate * pow(1.1f, getScrNum);
				++simuWalkNum;

				// ウサギがステージ内に収まっているか境界判定
				//assert(simuRabPos.x > 0 && simuRabPos.x < STAGE_WIDTH);
				//assert(simuRabPos.y > 0 && simuRabPos.y < STAGE_HEIGHT);
				if (simuRabPos.x < 0) simuRabPos.x = 0.0f;
				if (simuRabPos.x > STAGE_WIDTH) simuRabPos.x = float(STAGE_HEIGHT - 1);
				if (simuRabPos.y < 0) simuRabPos.y = 0.0f;
				if (simuRabPos.y > STAGE_HEIGHT) simuRabPos.y = float(STAGE_HEIGHT - 1);

				if (int(simuRabPos.x) >= int(tgtVec.x) &&
					int(simuRabPos.y) >= int(tgtVec.y)) {
					isReached = true;
				}

				// 計算は200ターンで打ち切り
				if (simuWalkNum > 200) {
					break;
				}
			}
		}

		// =====
		// 3. x: tgtの方が小さい y: tgtの方が大きい（左下イメージ）
		else if (dirVec.x < 0 && dirVec.y > 0)
		{
			bool isReached = false;

			// 仮想ウサギ座標がtarget座標に到達するまで、ループし続ける
			while (int(simuRabPos.x) > tgtVec.x&&
				int(simuRabPos.y) < tgtVec.y ||
				!isReached)
			{
				// 現在立っている地形を数値化
				assert((int)aStage.terrain(simuRabPos) > -1);
				float nowTerrainRate = convertTerrainToFloat(aStage.terrain(simuRabPos));

				simuRabPos.x += dirVec.x * nowTerrainRate * pow(1.1f, getScrNum);
				simuRabPos.y += dirVec.y * nowTerrainRate * pow(1.1f, getScrNum);
				++simuWalkNum;

				// ウサギがステージ内に収まっているか境界判定
				//assert(simuRabPos.x > 0 && simuRabPos.x < STAGE_WIDTH);
				//assert(simuRabPos.y > 0 && simuRabPos.y < STAGE_HEIGHT);
				if (simuRabPos.x < 0) simuRabPos.x = 0.0f;
				if (simuRabPos.x > STAGE_WIDTH) simuRabPos.x = float(STAGE_HEIGHT - 1);
				if (simuRabPos.y < 0) simuRabPos.y = 0.0f;
				if (simuRabPos.y > STAGE_HEIGHT) simuRabPos.y = float(STAGE_HEIGHT - 1);

				if (int(simuRabPos.x) <= int(tgtVec.x) &&
					int(simuRabPos.y) >= int(tgtVec.y)) {
					isReached = true;
				}

				// 計算は200ターンで打ち切り
				if (simuWalkNum > 200) {
					break;
				}
			}
		}

		// =====
		// 4. x: tgtの方が小さい y: tgtの方が小さい（左上イメージ）
		else if (dirVec.x < 0 && dirVec.y <= 0)
		{
			bool isReached = false;

			// 仮想ウサギ座標がtarget座標に到達するまで、ループし続ける
			while (int(simuRabPos.x) > tgtVec.x&&
				int(simuRabPos.y) > tgtVec.y ||
				!isReached)
			{
				// 現在立っている地形を数値化
				assert((int)aStage.terrain(simuRabPos) > -1);
				float nowTerrainRate = convertTerrainToFloat(aStage.terrain(simuRabPos));

				simuRabPos.x += dirVec.x * nowTerrainRate * pow(1.1f, getScrNum);
				simuRabPos.y += dirVec.y * nowTerrainRate * pow(1.1f, getScrNum);
				++simuWalkNum;

				// ウサギがステージ内に収まっているか境界判定
				//assert(simuRabPos.x > 0 && simuRabPos.x < STAGE_WIDTH);
				//assert(simuRabPos.y > 0 && simuRabPos.y < STAGE_HEIGHT);
				if (simuRabPos.x < 0) simuRabPos.x = 0.0f;
				if (simuRabPos.x > STAGE_WIDTH) simuRabPos.x = float(STAGE_HEIGHT - 1);
				if (simuRabPos.y < 0) simuRabPos.y = 0.0f;
				if (simuRabPos.y > STAGE_HEIGHT) simuRabPos.y = float(STAGE_HEIGHT - 1);

				if (int(simuRabPos.x) <= int(tgtVec.x) &&
					int(simuRabPos.y) <= int(tgtVec.y)) {
					isReached = true;
				}

				// 計算は200ターンで打ち切り
				if (simuWalkNum > 200) {
					break;
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
	/// 毎フレーム呼び出される処理
	/// @detail 移動先を決定して返します
	/// @param aStage 現在のステージ
	/// @return 移動の目標座標
	Vector2 Answer::getTargetPos(const Stage& aStage)
	{
		++debugTurnNum;
		auto rabPos = aStage.rabbit().pos();

		if (int(rabPos.x) == int(_RelayList[_OrderGetScrWithRelay[nowTarget] + 1].x) &&
			int(rabPos.y) == int(_RelayList[_OrderGetScrWithRelay[nowTarget] + 1].y)) {
			++nowTarget;
			if (nowTarget >= _OrderGetScrWithRelay.size()) {
				--nowTarget;
			}
			assert(_RelayList[_OrderGetScrWithRelay[nowTarget] + 1].x < STAGE_WIDTH + 1.0f);
			assert(_RelayList[_OrderGetScrWithRelay[nowTarget] + 1].y < STAGE_HEIGHT + 1.0f);
			return _RelayList[int(_OrderGetScrWithRelay[nowTarget]) + 1];
		}
		else {
			if (nowTarget >= _OrderGetScrWithRelay.size()) {
				--nowTarget;
			}
			assert(_RelayList[_OrderGetScrWithRelay[nowTarget] + 1].x < STAGE_WIDTH + 1.0f);
			assert(_RelayList[_OrderGetScrWithRelay[nowTarget] + 1].y < STAGE_HEIGHT + 1.0f);
			return _RelayList[int(_OrderGetScrWithRelay[nowTarget]) + 1];
		}
	}

	//------------------------------------------------------------------------------
	/// 各ステージ終了時に呼び出される処理
	/// @detail 各ステージに対する終了処理が必要ならここに書きます
	/// @param aStage 現在のステージ
	void Answer::finalize(const Stage& aStage)
	{
		// 順番を決めるリストはステージごとに空にする
		orderGetScr.clear();
		orderGetScr2nd.clear();
		_OrderGetScrWithRelay.clear();
		debugGreedySimu.clear();
		debug2ndGreedySimu.clear();
		nowTarget = 0;
		debugTurnNum = 0;
		_DijkstraGetScrNum = 0;
		STAGE_NUM++;
	}

} // namespace
// EOF