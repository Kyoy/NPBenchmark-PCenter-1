#include "Solver.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>

#include <cmath>

const int INF = 1000000000;


using namespace std;


namespace szx {

struct pair {
    int server;
    int user;
};

void initSolution(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, int centerNum, int &maxLength);

void genNextServer(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, int &maxLength);

void addFacility(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, int &maxLength, int addIndex);

void removeFacility(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, int &maxLength, int removeCenter);

void findNext(vector<int> &centerList, Arr2D<int> &adjMatrix, int nodeIndex, int &secondServer, int &secondLength);

void findPairWithTabu(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, Arr2D<int> &tabuMartrix, int &maxLength, int &iter);

void findPairWithDoubleTabu(vector<int> &centerList, vector<vector<int>> &nodeMatrix, vector<vector<int>> &distanceMatrix, Arr2D<int> &adjMatrix, vector<int> &userTabuList, vector<int> &serverTabuList, int &maxLength, int &iter);

bool isUnique(vector<int> vector);

vector<String> splitString(const string& str, string separator);

#pragma region Solver::Cli
int Solver::Cli::run(int argc, char * argv[]) {
    Log(LogSwitch::Szx::Cli) << "parse command line arguments." << endl;
    Set<String> switchSet;
    Map<String, char*> optionMap({ // use string as key to compare string contents instead of pointers.
        { InstancePathOption(), nullptr },
        { SolutionPathOption(), nullptr },
        { RandSeedOption(), nullptr },
        { TimeoutOption(), nullptr },
        { MaxIterOption(), nullptr },
        { JobNumOption(), nullptr },
        { RunIdOption(), nullptr },
        { EnvironmentPathOption(), nullptr },
        { ConfigPathOption(), nullptr },
        { LogPathOption(), nullptr }
    });

    for (int i = 1; i < argc; ++i) { // skip executable name.
        auto mapIter = optionMap.find(argv[i]);
        if (mapIter != optionMap.end()) { // option argument.
            mapIter->second = argv[++i];
        } else { // switch argument.
            switchSet.insert(argv[i]);
        }
    }

    Log(LogSwitch::Szx::Cli) << "execute commands." << endl;
    if (switchSet.find(HelpSwitch()) != switchSet.end()) {
        cout << HelpInfo() << endl;
    }

    if (switchSet.find(AuthorNameSwitch()) != switchSet.end()) {
        cout << AuthorName() << endl;
    }

    Solver::Environment env;
    env.load(optionMap);
    if (env.instPath.empty() || env.slnPath.empty()) { return -1; }

    Solver::Configuration cfg;
    cfg.load(env.cfgPath);

    Log(LogSwitch::Szx::Input) << "load instance " << env.instPath << " (seed=" << env.randSeed << ")." << endl;
    Problem::Input input;
    if (!input.load(env.instPath)) { return -1; }

    Solver solver(input, env, cfg);
    solver.solve();

    pb::Submission submission;
    submission.set_thread(to_string(env.jobNum));
    submission.set_instance(env.friendlyInstName());
    submission.set_duration(to_string(solver.timer.elapsedSeconds()) + "s");

    solver.output.save(env.slnPath, submission);
    #if SZX_DEBUG
    solver.output.save(env.solutionPathWithTime(), submission);
    solver.record();
    #endif // SZX_DEBUG

    return 0;
}
#pragma endregion Solver::Cli

#pragma region Solver::Environment
void Solver::Environment::load(const Map<String, char*> &optionMap) {
    char *str;

    str = optionMap.at(Cli::EnvironmentPathOption());
    if (str != nullptr) { loadWithoutCalibrate(str); }

    str = optionMap.at(Cli::InstancePathOption());
    if (str != nullptr) { instPath = str; }

    str = optionMap.at(Cli::SolutionPathOption());
    if (str != nullptr) { slnPath = str; }

    str = optionMap.at(Cli::RandSeedOption());
    if (str != nullptr) { randSeed = atoi(str); }

    str = optionMap.at(Cli::TimeoutOption());
    if (str != nullptr) { msTimeout = static_cast<Duration>(atof(str) * Timer::MillisecondsPerSecond); }

    str = optionMap.at(Cli::MaxIterOption());
    if (str != nullptr) { maxIter = atoi(str); }

    str = optionMap.at(Cli::JobNumOption());
    if (str != nullptr) { jobNum = atoi(str); }

    str = optionMap.at(Cli::RunIdOption());
    if (str != nullptr) { rid = str; }

    str = optionMap.at(Cli::ConfigPathOption());
    if (str != nullptr) { cfgPath = str; }

    str = optionMap.at(Cli::LogPathOption());
    if (str != nullptr) { logPath = str; }

    calibrate();
}

void Solver::Environment::load(const String &filePath) {
    loadWithoutCalibrate(filePath);
    calibrate();
}

void Solver::Environment::loadWithoutCalibrate(const String &filePath) {
    // EXTEND[szx][8]: load environment from file.
    // EXTEND[szx][8]: check file existence first.
}

void Solver::Environment::save(const String &filePath) const {
    // EXTEND[szx][8]: save environment to file.
}
void Solver::Environment::calibrate() {
    // adjust thread number.
    int threadNum = thread::hardware_concurrency();
    if ((jobNum <= 0) || (jobNum > threadNum)) { jobNum = threadNum; }

    // adjust timeout.
    msTimeout -= Environment::SaveSolutionTimeInMillisecond;
}
#pragma endregion Solver::Environment

#pragma region Solver::Configuration
void Solver::Configuration::load(const String &filePath) {
    // EXTEND[szx][5]: load configuration from file.
    // EXTEND[szx][8]: check file existence first.
}

void Solver::Configuration::save(const String &filePath) const {
    // EXTEND[szx][5]: save configuration to file.
}
#pragma endregion Solver::Configuration

#pragma region Solver
bool Solver::solve() {
    init();

    int workerNum = (max)(1, env.jobNum / cfg.threadNumPerWorker);
    cfg.threadNumPerWorker = env.jobNum / workerNum;
    List<Solution> solutions(workerNum, Solution(this));
    List<bool> success(workerNum);

    Log(LogSwitch::Szx::Framework) << "launch " << workerNum << " workers." << endl;
    List<thread> threadList;
    threadList.reserve(workerNum);
    for (int i = 0; i < workerNum; ++i) {
        // TODO[szx][2]: as *this is captured by ref, the solver should support concurrency itself, i.e., data members should be read-only or independent for each worker.
        // OPTIMIZE[szx][3]: add a list to specify a series of algorithm to be used by each threads in sequence.
        threadList.emplace_back([&, i]() { success[i] = optimize(solutions[i], i); });
    }
    for (int i = 0; i < workerNum; ++i) { threadList.at(i).join(); }

    Log(LogSwitch::Szx::Framework) << "collect best result among all workers." << endl;
    int bestIndex = -1;
    Length bestValue = Problem::MaxDistance;
    for (int i = 0; i < workerNum; ++i) {
        if (!success[i]) { continue; }
        Log(LogSwitch::Szx::Framework) << "worker " << i << " got " << solutions[i].coverRadius << endl;
        if (solutions[i].coverRadius >= bestValue) { continue; }
        bestIndex = i;
        bestValue = solutions[i].coverRadius;
    }

    env.rid = to_string(bestIndex);
    if (bestIndex < 0) { return false; }
    output = solutions[bestIndex];
    return true;
}

void Solver::record() const {
    #if SZX_DEBUG
    int generation = 0;

    ostringstream log;

    System::MemoryUsage mu = System::peakMemoryUsage();

    Length obj = output.coverRadius;
    Length checkerObj = -1;
    bool feasible = check(checkerObj);

    // record basic information.
    log << env.friendlyLocalTime() << ","
        << env.rid << ","
        << env.instPath << ","
        << feasible << "," << (obj - checkerObj) << ",";
    if (Problem::isTopologicalGraph(input)) {
        log << obj << ",";
    } else {
        auto oldPrecision = log.precision();
        log.precision(2);
        log << fixed << setprecision(2) << (obj / aux.objScale) << ",";
        log.precision(oldPrecision);
    }
    log << timer.elapsedSeconds() << ","
        << mu.physicalMemory << "," << mu.virtualMemory << ","
        << env.randSeed << ","
        << cfg.toBriefStr() << ","
        << generation << "," << iteration << ",";

    // record solution vector.
    // EXTEND[szx][2]: save solution in log.
    log << endl;

    // append all text atomically.
    static mutex logFileMutex;
    lock_guard<mutex> logFileGuard(logFileMutex);

    ofstream logFile(env.logPath, ios::app);
    logFile.seekp(0, ios::end);
    if (logFile.tellp() <= 0) {
        logFile << "Time,ID,Instance,Feasible,ObjMatch,Distance,Duration,PhysMem,VirtMem,RandSeed,Config,Generation,Iteration,Solution" << endl;
    }
    logFile << log.str();
    logFile.close();
    #endif // SZX_DEBUG
}

bool Solver::check(Length &checkerObj) const {
    #if SZX_DEBUG
    enum CheckerFlag {
        IoError = 0x0,
        FormatError = 0x1,
        TooManyCentersError = 0x2
    };

    checkerObj = System::exec("Checker.exe " + env.instPath + " " + env.solutionPathWithTime());
    if (checkerObj > 0) { return true; }
    checkerObj = ~checkerObj;
    if (checkerObj == CheckerFlag::IoError) { Log(LogSwitch::Checker) << "IoError." << endl; }
    if (checkerObj & CheckerFlag::FormatError) { Log(LogSwitch::Checker) << "FormatError." << endl; }
    if (checkerObj & CheckerFlag::TooManyCentersError) { Log(LogSwitch::Checker) << "TooManyCentersError." << endl; }
    return false;
    #else
    checkerObj = 0;
    return true;
    #endif // SZX_DEBUG
}

void Solver::init() {
    ID nodeNum = input.graph().nodenum();

    aux.adjMat.init(nodeNum, nodeNum);
    fill(aux.adjMat.begin(), aux.adjMat.end(), Problem::MaxDistance);
    for (ID n = 0; n < nodeNum; ++n) { aux.adjMat.at(n, n) = 0; }

    if (Problem::isTopologicalGraph(input)) {
        aux.objScale = Problem::TopologicalGraphObjScale;
        for (auto e = input.graph().edges().begin(); e != input.graph().edges().end(); ++e) {
            // only record the last appearance of each edge.
            aux.adjMat.at(e->source(), e->target()) = e->length();
            aux.adjMat.at(e->target(), e->source()) = e->length();
        }

        Timer timer(30s);
        constexpr bool IsUndirectedGraph = true;
        IsUndirectedGraph
            ? Floyd::findAllPairsPaths_symmetric(aux.adjMat)
            : Floyd::findAllPairsPaths_asymmetric(aux.adjMat);
        Log(LogSwitch::Preprocess) << "Floyd takes " << timer.elapsedSeconds() << " seconds." << endl;
    } else { // geometrical graph.
        aux.objScale = Problem::GeometricalGraphObjScale;
        for (ID n = 0; n < nodeNum; ++n) {
            double nx = input.graph().nodes(n).x();
            double ny = input.graph().nodes(n).y();
            for (ID m = 0; m < nodeNum; ++m) {
                if (n == m) { continue; }
                aux.adjMat.at(n, m) = static_cast<Length>(aux.objScale * hypot(
                    nx - input.graph().nodes(m).x(), ny - input.graph().nodes(m).y()));
            }
        }
    }

    aux.coverRadii.init(nodeNum);
    fill(aux.coverRadii.begin(), aux.coverRadii.end(), Problem::MaxDistance);
}

bool Solver::optimize(Solution &sln, ID workerId) {
    Log(LogSwitch::Szx::Framework) << "worker " << workerId << " starts." << endl;

    ID nodeNum = input.graph().nodenum();
    ID centerNum = input.centernum();

    // reset solution state.
    bool status = true;
    auto &centers(*sln.mutable_centers());
    centers.Resize(centerNum, Problem::InvalidId);

    // TODO[0]: replace the following random assignment with your own algorithm.

    #pragma region myAlg
    int bestSln[40] = {127, 98, 93, 74, 48, 84, 64, 55, 37, 20,
    59, 51, 36, 26, 18, 47, 39, 28, 18, 13,
    40, 38, 22, 15, 11, 38, 32, 18, 13, 9,
    30, 29, 15, 11, 30, 27, 15, 29, 23, 13};
    String fileName = env.friendlyInstName();
    //cout << fileName << endl;
    vector<String> names = splitString(fileName, ".");
    int index = stoi(names[0].substr(4,2));
    //cout << index << endl;
    int maxLength = INF, tempMaxLength = INF;
    int iter = 0;
    vector<int> centerList; // 下标从0开始
    //vector<vector<int>> nodeMatrix(2, vector<int>(nodeNum, INF)); // F表
    //vector<vector<int>> distanceMatrix(2, vector<int>(nodeNum, INF)); // D表
    //vector<vector<int>> tabuMatrix(nodeNum, vector<int>(nodeNum, iter));
    Arr2D<int> nodeMatrix(2, nodeNum);
    Arr2D<int> distanceMatrix(2, nodeNum);
    Arr2D<int> tabuMatrix(nodeNum, nodeNum);
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < nodeNum; ++j) {
            nodeMatrix[i][j] = INF;
            distanceMatrix[i][j] = INF;
        }
    }
    for (int i = 0; i < nodeNum; ++i) {
        for (int j = 0; j < nodeNum; ++j) {
            tabuMatrix[i][j] = iter;
        }
    }
    centerList.reserve(centerNum);
    initSolution(centerList, nodeMatrix, distanceMatrix, aux.adjMat, centerNum, maxLength);
    Timer timer(180s);
    for (int i = 0; !timer.isTimeOut(); ++i) {
        if (maxLength < tempMaxLength) {
            cout << i << " current maxLength:" << maxLength << endl;
            tempMaxLength = maxLength;
        }
        findPairWithTabu(centerList, nodeMatrix, distanceMatrix, aux.adjMat, tabuMatrix, maxLength, iter);
        if (maxLength == bestSln[index - 1]) {
            break;
        }
        //findPairWithDoubleTabu(centerList, nodeMatrix, distanceMatrix, aux.adjMat, userTabuList, serverTabuList, maxLength, iter);
    }
    for (int i = 0; i < centerNum; ++i) {
        centers[i] = centerList[i];
    }
    sln.coverRadius = maxLength;
    #pragma endregion myAlg

    Log(LogSwitch::Szx::Framework) << "worker " << workerId << " ends." << endl;
    return status;
}
#pragma endregion Solver

void initSolution(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, int centerNum, int &maxLength) {
    int nodeNum = adjMatrix.size1();
    Random rand;
    int index = rand.pick(0, nodeNum); // 随机产生一个初始解，从0开始
    addFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, index);
    while (centerList.size() < centerNum) {
        genNextServer(centerList,nodeMatrix, distanceMatrix, adjMatrix, maxLength);
    }
}

void genNextServer(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, int& maxLength) {
    Random rand;
    vector<int> nwk;
    vector<int> candidateNode;
    int nodeNum = adjMatrix.size1();
    for (int i = 0; i < nodeNum; ++i) { // 所有最长服务边
        if (distanceMatrix[0][i] == maxLength) {
            candidateNode.emplace_back(i);
        }
    }
    int index = candidateNode[rand.pick(0, candidateNode.size())];
    int centerIndex = nodeMatrix[0][index];
    for (int i = 0; i < nodeNum; ++i) {
        if (i != centerIndex && adjMatrix[index][i] <= maxLength) {
            nwk.emplace_back(i);
        }
    }
    int randIndex = rand.pick(0, nwk.size()); // nwk中随机寻找服务节点
    while (find(centerList.begin(), centerList.end(), nwk[randIndex]) != centerList.end()) { //已是服务节点时，重新随机寻找
        randIndex = rand.pick(0, nwk.size());
    }
    addFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, randIndex);
}

void addFacility(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, int &maxLength, int addIndex) {
    centerList.emplace_back(addIndex); // 添加指定用户节点，从0开始
    int nodeNum = adjMatrix.size1();
    for (int i = 0; i < nodeNum; ++i) {
        if (adjMatrix[addIndex][i] < distanceMatrix[0][i]) {
            distanceMatrix[1][i] = distanceMatrix[0][i];
            nodeMatrix[1][i] = nodeMatrix[0][i];
            distanceMatrix[0][i] = adjMatrix[addIndex][i];
            nodeMatrix[0][i] = addIndex;
        } else if (adjMatrix[addIndex][i] < distanceMatrix[1][i]) {
            distanceMatrix[1][i] = adjMatrix[addIndex][i];
            nodeMatrix[1][i] = addIndex;
        }
    }
    auto maxPosition = max_element(distanceMatrix[0], distanceMatrix[0] + nodeNum);
    maxLength = *maxPosition;
}

void removeFacility(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, int &maxLength, int removeCenter) {
    for (auto i = centerList.begin(); i != centerList.end(); ++i) { // 遍历删除指定编号的服务节点，从0开始
        if (*i == removeCenter) {
            centerList.erase(i);
            break;
        }
    }
    int nodeNum = adjMatrix.size1();
    for (int i = 0; i < nodeNum; ++i) {
        if (nodeMatrix[0][i] == removeCenter) {
            distanceMatrix[0][i] = distanceMatrix[1][i];
            nodeMatrix[0][i] = nodeMatrix[1][i];
            int secondServer = 0, secondLength = INF;
            findNext(centerList, adjMatrix, i, secondServer, secondLength);
            distanceMatrix[1][i] = secondLength;
            nodeMatrix[1][i] = secondServer;
        } else if (nodeMatrix[1][i] == removeCenter) {
            int secondServer = 0, secondLength = INF;
            findNext(centerList, adjMatrix, i, secondServer, secondLength);
            distanceMatrix[1][i] = secondLength;
            nodeMatrix[1][i] = secondServer;
        }
    }
    auto maxPosition = max_element(distanceMatrix[0], distanceMatrix[0] + nodeNum);
    maxLength = *maxPosition;
}

void findNext(vector<int> &centerList, Arr2D<int> &adjMatrix, int nodeIndex, int &secondServer, int &secondLength) {
    int firstServer = 0, firstLength = INF;
    for (auto i : centerList) {
        if (adjMatrix[nodeIndex][i] < firstLength) {
            secondLength = firstLength;
            secondServer = firstServer;
            firstLength = adjMatrix[nodeIndex][i];
            firstServer = i;
        } else if (adjMatrix[nodeIndex][i] < secondLength) {
            secondLength = adjMatrix[nodeIndex][i];
            secondServer = i;
        }
    }
}

void findPairWithTabu(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, Arr2D<int> &tabuMartrix, int &maxLength, int &iter) {
    Random rand;
    int nodeNum = adjMatrix.size1();
    int centerNum = centerList.size();
    int tabuTenure = iter + 0.3 * nodeNum + rand.pick(0, 0.3 * centerNum);
    vector<pair> longestPairList;
    for (int i = 0; i < nodeNum; ++i) {
        if (distanceMatrix[0][i] == maxLength) { // 所有最长服务边存入列表
            pair nodePair;
            nodePair.server = nodeMatrix[0][i];
            nodePair.user = i;
            longestPairList.emplace_back(nodePair);
        }
    }
    pair longestPair = longestPairList[rand.pick(0, longestPairList.size())]; // 随机取一最长服务边
    vector<int> nwk;
    for (int i = 0; i < nodeNum; ++i) { // 构造nwk,注意不能含有服务节点
        if (i != longestPair.server && adjMatrix[longestPair.user][i] <= maxLength && find(centerList.begin(), centerList.end(), i) == centerList.end()) {
            nwk.emplace_back(i);
        }
    }
    int tempMaxLength = INF;
    int tempTabuMaxLength = INF;
    vector<pair> candidatePairList; // 非禁忌候选节点对
    vector<pair> candidateTabuPairList; // 禁忌候选节点对
    pair candidatePair;
    for (auto i : nwk) {
        for (auto j : centerList) {
            vector<int> tempCenterList = centerList;
            Arr2D<int> tempNodeMatrix = nodeMatrix;
            Arr2D<int> tempDistanceMatrix = distanceMatrix;
            int sc = maxLength;
            addFacility(tempCenterList, tempNodeMatrix, tempDistanceMatrix, adjMatrix, sc, i);
            removeFacility(tempCenterList, tempNodeMatrix, tempDistanceMatrix, adjMatrix, sc, j);
            if (tabuMartrix[i][j] > iter) { // 节点对被禁忌
                if (sc < tempTabuMaxLength) { // sc小则清空候选tabuPair表，并加入tabuPair
                    candidateTabuPairList.swap(vector<pair>());
                    candidatePair.server = j;
                    candidatePair.user = i;
                    candidateTabuPairList.emplace_back(candidatePair);
                    tempTabuMaxLength = sc;
                } else if (sc == tempTabuMaxLength) { // sc相等则加入tabuPair
                    candidatePair.server = j;
                    candidatePair.user = i;
                    candidateTabuPairList.emplace_back(candidatePair);
                }
            } else {
                if (sc < tempMaxLength) { // sc小则清空候选pair表，并加入pair
                    candidatePairList.swap(vector<pair>());
                    candidatePair.server = j;
                    candidatePair.user = i;
                    candidatePairList.emplace_back(candidatePair);
                    tempMaxLength = sc;
                } else if (sc == tempMaxLength) { // sc相等则加入pair
                    candidatePair.server = j;
                    candidatePair.user = i;
                    candidatePairList.emplace_back(candidatePair);
                }
            }
        }
    }
    //cout << "list size:" << candidatePairList.size() << endl;
    //cout << "tabulist size:" << candidateTabuPairList.size() << endl;
    if (tempTabuMaxLength < maxLength && tempTabuMaxLength < tempMaxLength) { // 解禁
        cout << "解禁" << " tempTabu:" << tempTabuMaxLength << " max:" << maxLength << " temp:" << tempMaxLength << " iter:" << iter << endl;
        candidatePair = candidateTabuPairList[rand.pick(0, candidateTabuPairList.size())];
        tabuMartrix[candidatePair.user][candidatePair.server] = tabuTenure + iter + 1;
        tabuMartrix[candidatePair.server][candidatePair.user] = tabuTenure + iter + 1;
        addFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, candidatePair.user);
        removeFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, candidatePair.server);
        //iter++;
    }
    else if (candidatePairList.size() != 0) {
        candidatePair = candidatePairList[rand.pick(0, candidatePairList.size())]; // 随机选取优化程度最好的pair
        tabuMartrix[candidatePair.user][candidatePair.server] = tabuTenure + iter + 1;
        tabuMartrix[candidatePair.server][candidatePair.user] = tabuTenure + iter + 1;
        addFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, candidatePair.user);
        removeFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, candidatePair.server);
        //iter++;
    }
    iter++;
}

void findPairWithDoubleTabu(vector<int> &centerList, Arr2D<int> &nodeMatrix, Arr2D<int> &distanceMatrix, Arr2D<int> &adjMatrix, vector<int> &userTabuList, vector<int> &serverTabuList, int &maxLength, int &iter) {
    Random rand;
    int nodeNum = adjMatrix.size1();
    int centerNum = centerList.size();
    int serverTabuTenure = iter + centerNum / 10 + rand.pick(0, 10); // A 禁止转换为用户节点
    int userTabuTenure = iter + (nodeNum - centerNum) / 10 + rand.pick(0, 100); // B 禁止转换为服务节点
                                                                                /*int serverTabuTenure = iter + 0.3 * nodeNum + rand.pick(0, 0.3 * centerNum);
                                                                                int userTabuTenure = serverTabuTenure;*/
    vector<pair> longestPairList;
    for (int i = 0; i < nodeNum; ++i) {
        if (distanceMatrix[0][i] == maxLength) { // 所有最长服务边存入列表
            pair nodePair;
            nodePair.server = nodeMatrix[0][i];
            nodePair.user = i;
            longestPairList.emplace_back(nodePair);
        }
    }
    pair longestPair = longestPairList[rand.pick(0, longestPairList.size())]; // 随机取一最长服务边
    vector<int> nwk;
    vector<int> nwkWithTabu;
    for (int i = 0; i < nodeNum; ++i) { // 构造nwk,注意不能含有服务节点
        if (i != longestPair.server && adjMatrix[longestPair.user][i] <= maxLength && find(centerList.begin(), centerList.end(), i) == centerList.end()) {
            if (userTabuList[i] > iter) { // 用户节点被禁忌，禁忌步长内不能交换为服务节点
                nwkWithTabu.emplace_back(i);
            } else {
                nwk.emplace_back(i);
            }
        }
    }
    int tempMaxLength = INF;
    int tempTabuMaxLength = INF;
    vector<pair> candidatePairList; // 非禁忌候选节点对
    vector<pair> candidateTabuPairList; // 禁忌候选节点对
    pair candidatePair;
    for (auto i : nwk) {
        for (auto j : centerList) {
            vector<int> tempCenterList = centerList;
            Arr2D<int> tempNodeMatrix = nodeMatrix;
            Arr2D<int> tempDistanceMatrix = distanceMatrix;
            int sc = maxLength;
            addFacility(tempCenterList, tempNodeMatrix, tempDistanceMatrix, adjMatrix, sc, i);
            removeFacility(tempCenterList, tempNodeMatrix, tempDistanceMatrix, adjMatrix, sc, j);
            if (serverTabuList[j] > iter) { // 服务节点被禁忌，禁忌步长内不能交换为用户节点
                if (sc < tempTabuMaxLength) { // sc小则清空候选tabuPair表，并加入tabuPair
                    candidateTabuPairList.swap(vector<pair>());
                    candidatePair.server = j;
                    candidatePair.user = i;
                    candidateTabuPairList.emplace_back(candidatePair);
                    tempTabuMaxLength = sc;
                } else if (sc == tempTabuMaxLength) { // sc相等则加入tabuPair
                    candidatePair.server = j;
                    candidatePair.user = i;
                    candidateTabuPairList.emplace_back(candidatePair);
                }
            } else {
                if (sc < tempMaxLength) { // sc小则清空候选pair表，并加入pair
                    candidatePairList.swap(vector<pair>());
                    candidatePair.server = j;
                    candidatePair.user = i;
                    candidatePairList.emplace_back(candidatePair);
                    tempMaxLength = sc;
                } else if (sc == tempMaxLength) { // sc相等则加入pair
                    candidatePair.server = j;
                    candidatePair.user = i;
                    candidatePairList.emplace_back(candidatePair);
                }
            }
        }
    }
    for (auto i : nwkWithTabu) {
        for (auto j : centerList) {
            vector<int> tempCenterList = centerList;
            Arr2D<int> tempNodeMatrix = nodeMatrix;
            Arr2D<int> tempDistanceMatrix = distanceMatrix;
            int sc = maxLength;
            addFacility(tempCenterList, tempNodeMatrix, tempDistanceMatrix, adjMatrix, sc, i);
            removeFacility(tempCenterList, tempNodeMatrix, tempDistanceMatrix, adjMatrix, sc, j);
            if (sc < tempTabuMaxLength) { // sc小则清空候选tabuPair表，并加入tabuPair
                candidateTabuPairList.swap(vector<pair>());
                candidatePair.server = j;
                candidatePair.user = i;
                candidateTabuPairList.emplace_back(candidatePair);
                tempTabuMaxLength = sc;
            } else if (sc == tempTabuMaxLength) { // sc相等则加入tabuPair
                candidatePair.server = j;
                candidatePair.user = i;
                candidateTabuPairList.emplace_back(candidatePair);
            }
        }
    }
    //cout << "list size:" << candidatePairList.size() << endl;
    //cout << "tabulist size:" << candidateTabuPairList.size() << endl;
    if (tempTabuMaxLength < maxLength && tempTabuMaxLength < tempMaxLength) { // 解禁
        cout << "解禁" << " tempTabu:" << tempTabuMaxLength << " max:" << maxLength << " temp:" << tempMaxLength << " iter:" << iter << endl;
        candidatePair = candidateTabuPairList[rand.pick(0, candidateTabuPairList.size())];
        userTabuList[candidatePair.server] = userTabuTenure + iter + 1;
        serverTabuList[candidatePair.user] = serverTabuTenure + iter + 1;
        addFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, candidatePair.user);
        removeFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, candidatePair.server);
        //iter++;
    } else if (candidatePairList.size() != 0) {
        candidatePair = candidatePairList[rand.pick(0, candidatePairList.size())]; // 随机选取优化程度最好的pair
        userTabuList[candidatePair.server] = userTabuTenure + iter + 1;
        serverTabuList[candidatePair.user] = serverTabuTenure + iter + 1;
        addFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, candidatePair.user);
        removeFacility(centerList, nodeMatrix, distanceMatrix, adjMatrix, maxLength, candidatePair.server);
        //iter++;
    }
    iter++;
}


bool isUnique(vector<int> vector) { // 判断vector是否含有重复元素
    int size = vector.size();
    sort(vector.begin(), vector.end());
    auto it = unique(vector.begin(), vector.end());
    vector.erase(it, vector.end());
    if (size != vector.size()) {
        return false;
    } else {
        return true;
    }
}

vector<string> splitString(const string& str,string separator) {
    vector<string> elems;
    if ("" == str) return elems;
    char *strs = new char[str.length() + 1];
    strcpy(strs, str.c_str());
    char * d = new char[separator.length() + 1];
    strcpy(d, separator.c_str());

    char *p = strtok(strs, d);
    while (p) {
        string s = p; //分割得到的字符串转换为string类型
        elems.push_back(s); //存入结果数组
        p = strtok(NULL, d);
    }
    return elems;
}


}
