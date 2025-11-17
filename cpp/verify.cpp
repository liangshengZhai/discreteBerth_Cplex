#include <ilcplex/ilocplex.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>
#include <cstring>
#include "modelParam.h"

using namespace std;

// 集中管理数据输入和输出路径，只需改这里即可
static const std::string INPUT_BASE = "data/example_L12/params_output"; // 不带扩展名的前缀
static const std::string OUTPUT_DIR = "output/output_L12";               // 输出目录

// 递归创建目录（等价于 mkdir -p）
static bool mkdir_p(const std::string& dirPath) {
    if (dirPath.empty()) return true;
    std::string path;
    for (size_t i = 0; i < dirPath.size(); ++i) {
        char c = dirPath[i];
        path.push_back(c);
        if (c == '/' || i == dirPath.size() - 1) {
            if (!path.empty() && path != "/" && path != "./") {
                struct stat st;
                if (stat(path.c_str(), &st) != 0) {
                    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
                        std::cerr << "创建目录失败: " << path << ", 错误: " << std::strerror(errno) << std::endl;
                        return false;
                    }
                } else if (!S_ISDIR(st.st_mode)) {
                    std::cerr << "路径存在但不是目录: " << path << std::endl;
                    return false;
                }
            }
        }
    }
    return true;
}

// 数据初始化已提取为 data_init.cpp -> setParams()

// 主函数：构建并求解模型
int main() {
    
    //初始化CPLEX环境和模型
    IloEnv env;
    IloModel model(env);
    try {
    // 1. 读取模型参数（从 data/ 下的 CSV 文件）
    auto loadParamsFromCSV = [](const std::string &baseName) -> ModelParams {
        ModelParams params;
        // try to read general
        std::ifstream ifs_gen(baseName + "_general.csv");
        if (!ifs_gen.is_open()) {
            std::cerr << "无法打开 general 参数文件: " << baseName + "_general.csv" << std::endl;
            // Return empty/default params (caller should handle)
            return params;
        }
        std::string line;
        // skip header
        std::getline(ifs_gen, line);
        while (std::getline(ifs_gen, line)) {
            if (line.empty()) continue;
            std::istringstream ss(line);
            std::string key, val;
            if (!std::getline(ss, key, ',')) continue;
            if (!std::getline(ss, val)) continue;
            try {
                if (key == "numBerths") params.numBerths = std::stoi(val);
                else if (key == "numRows") params.numRows = std::stoi(val);
                else if (key == "numSlotsPerRow") params.numSlotsPerRow = std::stoi(val);
                else if (key == "numShips") params.numShips = std::stoi(val);
                else if (key == "planningHorizon") params.planningHorizon = std::stod(val);
                else if (key == "numShipK") params.numShipK = std::stoi(val);
                else if (key == "width") params.width = std::stod(val);
                else if (key == "relativeHeight") params.relativeHeight = std::stod(val);
                else if (key == "alpha") params.alpha = std::stod(val);
                else if (key == "beta") params.beta = std::stod(val);
            } catch (...) {
                // ignore parse errors per-value
            }
        }
        ifs_gen.close();

        // allocate containers based on general
        if (params.numShips <= 0) params.numShips = 0;
        if (params.numBerths <= 0) params.numBerths = 0;
        if (params.numRows <= 0) params.numRows = 0;
        if (params.numSlotsPerRow <= 0) params.numSlotsPerRow = 0;
        if (params.numShipK <= 0) params.numShipK = 0;

        params.arrivalTime.assign(params.numShips, 0.0);
        params.cargoWeight.assign(params.numShips, 0.0);
        params.cargoDensity.assign(params.numShips, std::vector<double>(params.numShipK, 0.0));
        params.maxResponseAngle.assign(params.numShips, std::vector<double>(params.numShipK, 0.0));
        params.requiredSlots.assign(params.numShips, std::vector<int>(params.numShipK, 0));
        params.unloadingSpeed.assign(params.numShips, std::vector<std::vector<double>>(params.numBerths, std::vector<double>(params.numShipK, 0.0)));
        params.storageCost.assign(params.numShips, std::vector<std::vector<double>>(params.numShipK, std::vector<double>(params.numRows, 0.0)));
        params.transshipmentCost.assign(params.numBerths, std::vector<std::vector<double>>(params.numRows, std::vector<double>(params.numSlotsPerRow, 0.0)));

        // helper to parse CSV lines
        auto parse_two = [&](const std::string &filename, std::function<void(int,double)> fn){
            std::ifstream ifs(filename);
            if(!ifs.is_open()) return;
            std::string h; std::getline(ifs,h);
            std::string l;
            while(std::getline(ifs,l)){
                if(l.empty()) continue;
                std::istringstream ss(l);
                std::string a,b;
                if(!std::getline(ss,a,',')) continue;
                if(!std::getline(ss,b)) continue;
                try{ fn(std::stoi(a), std::stod(b)); }catch(...){}
            }
        };

        // arrival
        parse_two(baseName + "_arrival.csv", [&](int s, double v){ if(s>=0 && s < params.numShips) params.arrivalTime[s]=v; });
        // cargoWeight
        parse_two(baseName + "_cargoWeight.csv", [&](int s, double v){ if(s>=0 && s < params.numShips) params.cargoWeight[s]=v; });

        // cargoDensity (ship,k,value)
        {
            std::ifstream ifs(baseName + "_cargoDensity.csv");
            if(ifs.is_open()){
                std::string h; std::getline(ifs,h);
                std::string l;
                while(std::getline(ifs,l)){
                    if(l.empty()) continue;
                    std::istringstream ss(l);
                    std::string s,k,v;
                    if(!std::getline(ss,s,',')) continue;
                    if(!std::getline(ss,k,',')) continue;
                    if(!std::getline(ss,v)) continue;
                    try{ int si=std::stoi(s), ki=std::stoi(k); double dv=std::stod(v); if(si>=0 && si<params.numShips && ki>=0 && ki<params.numShipK) params.cargoDensity[si][ki]=dv; }catch(...){}
                }
            }
        }

        // maxResponseAngle
        {
            std::ifstream ifs(baseName + "_maxResponseAngle.csv");
            if(ifs.is_open()){
                std::string h; std::getline(ifs,h);
                std::string l;
                while(std::getline(ifs,l)){
                    if(l.empty()) continue;
                    std::istringstream ss(l);
                    std::string s,k,v;
                    if(!std::getline(ss,s,',')) continue;
                    if(!std::getline(ss,k,',')) continue;
                    if(!std::getline(ss,v)) continue;
                    try{ int si=std::stoi(s), ki=std::stoi(k); double dv=std::stod(v); if(si>=0 && si<params.numShips && ki>=0 && ki<params.numShipK) params.maxResponseAngle[si][ki]=dv; }catch(...){}
                }
            }
        }

        // requiredSlots
        {
            std::ifstream ifs(baseName + "_requiredSlots.csv");
            if(ifs.is_open()){
                std::string h; std::getline(ifs,h);
                std::string l;
                while(std::getline(ifs,l)){
                    if(l.empty()) continue;
                    std::istringstream ss(l);
                    std::string s,k,v;
                    if(!std::getline(ss,s,',')) continue;
                    if(!std::getline(ss,k,',')) continue;
                    if(!std::getline(ss,v)) continue;
                    try{ int si=std::stoi(s), ki=std::stoi(k); int iv=std::stoi(v); if(si>=0 && si<params.numShips && ki>=0 && ki<params.numShipK) params.requiredSlots[si][ki]=iv; }catch(...){}
                }
            }
        }

        // unloadingSpeed (ship,berth,k,value)
        {
            std::ifstream ifs(baseName + "_unloadingSpeed.csv");
            if(ifs.is_open()){
                std::string h; std::getline(ifs,h);
                std::string l;
                while(std::getline(ifs,l)){
                    if(l.empty()) continue;
                    std::istringstream ss(l);
                    std::string s,b,k,v;
                    if(!std::getline(ss,s,',')) continue;
                    if(!std::getline(ss,b,',')) continue;
                    if(!std::getline(ss,k,',')) continue;
                    if(!std::getline(ss,v)) continue;
                    try{ int si=std::stoi(s), bi=std::stoi(b), ki=std::stoi(k); double dv=std::stod(v); if(si>=0 && si<params.numShips && bi>=0 && bi<params.numBerths && ki>=0 && ki<params.numShipK) params.unloadingSpeed[si][bi][ki]=dv; }catch(...){}
                }
            }
        }

        // transshipmentCost (berth,row,slot,value)
        {
            std::ifstream ifs(baseName + "_transshipmentCost.csv");
            if(ifs.is_open()){
                std::string h; std::getline(ifs,h);
                std::string l;
                while(std::getline(ifs,l)){
                    if(l.empty()) continue;
                    std::istringstream ss(l);
                    std::string b,r,v,val;
                    if(!std::getline(ss,b,',')) continue;
                    if(!std::getline(ss,r,',')) continue;
                    if(!std::getline(ss,v,',')) continue;
                    if(!std::getline(ss,val)) continue;
                    try{ int bi=std::stoi(b), ri=std::stoi(r), vi=std::stoi(v); double dv=std::stod(val); if(bi>=0 && bi<params.numBerths && ri>=0 && ri<params.numRows && vi>=0 && vi<params.numSlotsPerRow) params.transshipmentCost[bi][ri][vi]=dv; }catch(...){}
                }
            }
        }

        // storageCost (ship,k,row,value)
        {
            std::ifstream ifs(baseName + "_storageCost.csv");
            if(ifs.is_open()){
                std::string h; std::getline(ifs,h);
                std::string l;
                while(std::getline(ifs,l)){
                    if(l.empty()) continue;
                    std::istringstream ss(l);
                    std::string s,k,r,v;
                    if(!std::getline(ss,s,',')) continue;
                    if(!std::getline(ss,k,',')) continue;
                    if(!std::getline(ss,r,',')) continue;
                    if(!std::getline(ss,v)) continue;
                    try{ int si=std::stoi(s), ki=std::stoi(k), ri=std::stoi(r); double dv=std::stod(v); if(si>=0 && si<params.numShips && ki>=0 && ki<params.numShipK && ri>=0 && ri<params.numRows) params.storageCost[si][ki][ri]=dv; }catch(...){}
                }
            }
        }

        return params;
    };


    
    ModelParams params = loadParamsFromCSV(INPUT_BASE);
    // Diagnostic print: verify that params were loaded correctly
    std::cout << "[DEBUG] Loaded params: numBerths=" << params.numBerths
              << " numRows=" << params.numRows
              << " numSlotsPerRow=" << params.numSlotsPerRow
              << " numShips=" << params.numShips
              << " numShipK=" << params.numShipK
              << " planningHorizon=" << params.planningHorizon << std::endl;
    std::cout << "[DEBUG] arrivalTime.size=" << params.arrivalTime.size()
              << " cargoWeight.size=" << params.cargoWeight.size()
              << " unloadingSpeed.size=" << params.unloadingSpeed.size()
              << " transshipmentCost.size=" << params.transshipmentCost.size()
              << " storageCost.size=" << params.storageCost.size() << std::endl;


    
        // 3. 定义决策变量
        // x_skrv: 船舶s的货物是否分配到行r的槽v
        IloArray<IloArray<IloArray<IloArray<IloBoolVar>>>> x(env);
        // h_skrv: 船舶s的货物是否结束于行r的槽v
        IloArray<IloArray<IloArray<IloArray<IloBoolVar>>>> h(env);
        // f_skr: 船舶s的货物是否分配到行r
        IloArray<IloArray<IloArray<IloBoolVar>>> f(env);
        // y_st: 船舶s的卸载是否在船舶t之前
        IloArray<IloArray<IloBoolVar>> y(env);
        // z_sb: 船舶s是否分配到泊位b
        IloArray<IloArray<IloBoolVar>> z(env);
        // q_skt
        IloArray<IloArray<IloArray<IloBoolVar>>>q(env);
        // e_s: 船舶s的卸载开始时间
        IloArray<IloNumVar> e(env);
        //e_sk:船舶s 货舱k的卸货时间
        IloArray<IloArray<IloNumVar>> e_sk(env);
        

        // 初始化变量
        for (int s = 0; s < params.numShips; s++) {
            // 初始化z_sb
            IloArray<IloBoolVar> z_s(env, params.numBerths);
            for (int b = 0; b < params.numBerths; b++) {
                string z_name = "z_" + to_string(s) + "_" + to_string(b);
                z_s[b]=IloBoolVar(env,z_name.c_str());
            }
            z.add(z_s);

            //初始化q_skt
            IloArray<IloArray<IloBoolVar>> q_s(env, params.numShipK);
            for(int k = 0 ; k< params.numShipK;k++){
                q_s[k] = IloArray<IloBoolVar>(env,params.numShipK);
                for(int t = 0;  t < params.numShipK;t++){
                    string q_name = "z_"+to_string(s) + "_" + to_string(k)+"_"+to_string(t);
                    q_s[k][t] = IloBoolVar(env,q_name.c_str());
                }
            }
            q.add(q_s);
            
            // 初始化y_st
            IloArray<IloBoolVar> y_s(env,params.numShips);
            for (int t = 0; t < params.numShips; t++) {
                string y_name = "y_" + to_string(s) +"_"+to_string(t);
                y_s[t]= IloBoolVar(env,y_name.c_str());
            }
            y.add(y_s);
            
            // 初始化f_skr
            IloArray<IloArray<IloBoolVar>> f_s(env,params.numShipK);
            for(int k = 0 ; k < params.numShipK;k++){
                f_s[k] = IloArray<IloBoolVar>(env,params.numRows);
                for (int r = 0; r < params.numRows; r++) {
                    string f_name = "f_" + to_string(s) + "_" + to_string(r)+ "_"+ to_string(k);
                    f_s[k][r] = IloBoolVar(env,f_name.c_str());
                }
            }
            f.add(f_s);
            
            // 初始化e_s
            e.add(IloNumVar(env, params.arrivalTime[s], params.planningHorizon));

            // 初始化e_sk（货舱卸载开始时间）：不强制上界为规划期，允许超过 planningHorizon
            IloArray<IloNumVar> e_s(env,params.numShipK);
            for(int k = 0 ; k < params.numShipK;k++){
                e_s[k]=IloNumVar(env, params.arrivalTime[s], IloInfinity);
            }
            e_sk.add(e_s);
            // 初始化x_skrv和h_skrv

            
            // 初始化x[s]（二维数组）
            IloArray<IloArray<IloArray<IloBoolVar>>> x_s(env,params.numShipK);
            for(int k = 0; k< params.numShipK;k++){
                x_s[k] = IloArray<IloArray<IloBoolVar>>(env, params.numRows);
                for (int r = 0; r < params.numRows; r++) {
                    x_s[k][r] = IloArray<IloBoolVar>(env, params.numSlotsPerRow);
                    // 初始化x[s][r][v]（变量）
                    for (int v = 0; v < params.numSlotsPerRow; v++) {
                        string x_name = "x_" + to_string(s) + "_"+to_string(k)+"_" + to_string(r) + "_" + to_string(v);
                        x_s[k][r][v] = IloBoolVar(env, x_name.c_str());
                    }
                }
            }
            // 将x_s添加到x数组
            x.add(x_s);
            // 同理初始化h数组
            IloArray<IloArray<IloArray<IloBoolVar>>> h_s(env,params.numShipK);
            // h_s = IloArray<IloArray<IloBoolVar>>(env, params.numRows);
            for(int k = 0 ; k< params.numShipK;k++){
                h_s[k] = IloArray<IloArray<IloBoolVar>>(env, params.numRows);
                for (int r = 0; r < params.numRows; r++) {
                    h_s[k][r] = IloArray<IloBoolVar>(env, params.numSlotsPerRow);
                    for (int v = 0; v < params.numSlotsPerRow; v++) {
                        string h_name = "h_" + to_string(s) + "_" +to_string(k)+"_"+ to_string(r) + "_" + to_string(v);
                        h_s[k][r][v] = IloBoolVar(env, h_name.c_str());
                    }
                }
            }
            h.add(h_s);
            
        }
        
        // 4. 构建目标函数：最小化总转运成本、存储成本和靠泊时间
        IloExpr objExpr(env);
        // 总转运成本
        for (int s = 0; s < params.numShips; s++) {
            for(int k =0 ;k < params.numShipK;k++){
                for (int b = 0; b < params.numBerths; b++) {
                    for (int r = 0; r < params.numRows; r++) {
                        for (int v = 0; v < params.numSlotsPerRow; v++) {
                            // int slotIndex = r * params.numSlotsPerRow + v;
                            objExpr += params.transshipmentCost[b][r][v] * 
                                    params.cargoWeight[s] / (params.requiredSlots[s][k]*params.numShipK) * 
                                    x[s][k][r][v] * z[s][b];
                        }
                    }
                }
            }
        }
        
        // 总存储成本
        for (int s = 0; s < params.numShips; s++) {
            for(int k =0 ; k <params.numShipK;k++){
                for (int r = 0; r < params.numRows; r++) {
                    for (int v = 0; v < params.numSlotsPerRow; v++) {
                        objExpr += params.storageCost[s][k][r] * x[s][k][r][v];
                    }
                }
            }
        }


       IloExpr berthTime(env);
        // 正确计算每艘船的靠泊时间（考虑泊位分配）
        for (int s = 0; s < params.numShips; s++) {
            IloExpr singleBerth(env);
            singleBerth += e[s] - params.arrivalTime[s];
    
            for(int k =0 ; k< params.numShipK;k++){
                // 仅累加分配泊位的卸载时间
                for (int b = 0; b < params.numBerths; b++) {
                    double speed = params.unloadingSpeed[s][b][k];
                    if (speed <= 0) speed = 1.0; // 防除零
                    singleBerth += (params.cargoWeight[s] / (speed * params.numShipK)) * z[s][b];
                }
            }
            berthTime += singleBerth;
            singleBerth.end();
        }
        // 应用权重
        objExpr = params.alpha * objExpr + params.beta * berthTime; // 注意：目标函数公式需根据文档调整权重应用方式
        
        model.add(IloMinimize(env, objExpr));
        berthTime.end();
        objExpr.end();
        
        // 5. 添加约束条件
        
        //约束(3.8): 每艘船分配到一个泊位
        for (int s = 0; s < params.numShips; s++) {
            IloExpr con(env);
            for (int b = 0; b < params.numBerths; b++) {
                con += z[s][b];
            }
            model.add(con == 1);
            con.end();
        }
        
        // 约束(3.11): 每艘船的每个舱占用足够的槽数
        for (int s = 0; s < params.numShips; s++) {
            for(int k = 0 ; k< params.numShipK;k++){
                IloExpr con(env);
                for (int r = 0; r < params.numRows; r++) {
                    for (int v = 0; v < params.numSlotsPerRow; v++) {
                        con += x[s][k][r][v];
                    }
                }
                // cout<<params.requiredSlots[s]<<endl;
                model.add(con == params.requiredSlots[s][k]);
                con.end();
            }
        }
        
        // 约束(3.12): 每个槽最多放一种货物（跨船舶 s 和货舱 k，总和 <= 1）
        // 之前的实现仅对每个 k 单独约束，允许相同槽被不同 k 的货物占用，造成重复占用的问题。
        for (int r = 0; r < params.numRows; r++) {
            for (int v = 0; v < params.numSlotsPerRow; v++) {
                IloExpr con(env);
                for (int s = 0; s < params.numShips; s++) {
                    for (int k = 0; k < params.numShipK; k++) {
                        con += x[s][k][r][v];
                    }
                }
                model.add(con <= 1);
                con.end();
            }
        }
        
        // 约束(3.13): 每艘船的货物存储在同一行
        for (int s = 0; s < params.numShips; s++) {
            for(int k = 0 ; k < params.numShipK;k++){
                IloExpr con(env);
                for (int r = 0; r < params.numRows; r++) {
                    con += f[s][k][r];
                }
                model.add(con == 1);
                con.end();
            }
        }
        
        // 约束(3.14): x_srv与f_sr的关联
        for (int s = 0; s < params.numShips; s++) {
            for(int k =0 ; k <params.numShipK;k++){
                for (int r = 0; r < params.numRows; r++) {
                    IloExpr con(env);
                    for (int v = 0; v < params.numSlotsPerRow; v++) {
                        con += x[s][k][r][v];
                    }
                    model.add(con <= params.numSlotsPerRow * f[s][k][r]);
                    con.end();
                }
            }
        }
        
        // 约束(12)-(14): 存储槽的连续性（简化实现，完整逻辑需按文档详细处理）
        for (int s = 0; s < params.numShips; s++) {
            for(int k =0; k< params.numShipK;k++){
                for (int r = 0; r < params.numRows; r++) {
                    IloExpr con(env);
                    for (int v = 0; v < params.numSlotsPerRow; v++) {
                        con += h[s][k][r][v];
                    }
                    model.add(con == f[s][k][r]);
                    con.end();
                    
                    // 约束(13): 最后一个槽的h_srv约束
                    model.add(x[s][k][r][params.numSlotsPerRow-1] <= h[s][k][r][params.numSlotsPerRow-1]);
                    
                    // 约束(14): 中间槽的连续性约束
                    for (int v = 0; v < params.numSlotsPerRow-1; v++) {
                        model.add(x[s][k][r][v] - x[s][k][r][v+1] <= h[s][k][r][v]);
                    }
                }
            }
        }
        
        //约束船舱卸货顺序
        for(int s = 0; s< params.numShips;s++){
            for(int k = 0 ; k< params.numShipK;k++){
                // IloExpr con(env);
                model.add(e[s] <= e_sk[s][k]);
            }
        }

        for(int s = 0; s<params.numShips;s++){
            for(int k = 0 ; k <params.numShipK;k++){
                for(int t = 0 ; t <params.numShipK;t++){
                    if( t==k){
                        continue;
                    }
                    model.add(q[s][k][t]+q[s][t][k] -1 ==0);
                    for(int b = 0 ; b <params.numBerths;b++){
                        // Big-M constraint to enforce unloading order for compartments k and t on ship s
                        // only when ship s is assigned to berth b (activate with z[s][b]).
                        // If q[s][k][t] == 1 AND z[s][b] == 1 then:
                        //   e_sk[s][k] + duration_kb <= e_sk[s][t]
                        // Linearized as:
                        //   e_sk[s][k] + duration_kb - e_sk[s][t] <= M * (2 - q[s][k][t] - z[s][b])
                        // For the chosen berth (z=1) and q=1, RHS=0 (binding). Otherwise RHS is large and non-binding.
                        double duration_kb = params.cargoWeight[s] / (params.numShipK * params.unloadingSpeed[s][b][k]);
                        double Mbig = 10000.0; // large constant
                        model.add(e_sk[s][k] + duration_kb - e_sk[s][t] <= Mbig * (2 - q[s][k][t] - z[s][b]));
                    }
                }
            }
        }
    // 6. 线性化处理（约束24-36和39-45）
        // 此处需完整实现线性化逻辑，以下为简化示例
        IloArray<IloArray<IloArray<IloBoolVar>>> omega(env);
        IloArray<IloArray<IloArray<IloBoolVar>>> lambda(env);
        IloArray<IloArray<IloArray<IloBoolVar>>> mu(env);
        IloArray<IloArray<IloArray<IloNumVar>>> zeta(env);
        IloArray<IloArray<IloArray<IloNumVar>>> eta(env);
        
        // 初始化线性化变量
        for (int s = 0; s < params.numShips; s++) {
            // 初始化s维度的数组
            IloArray<IloArray<IloBoolVar>> omega_s(env, params.numShips);
            IloArray<IloArray<IloBoolVar>> lambda_s(env, params.numShips);
            IloArray<IloArray<IloBoolVar>> mu_s(env, params.numShips);
            IloArray<IloArray<IloNumVar>> zeta_s(env, params.numShips);
            IloArray<IloArray<IloNumVar>> eta_s(env, params.numShips);
            
            for (int t = 0; t < params.numShips; t++) {
                // 初始化t维度的数组
                IloArray<IloBoolVar> omega_st(env, params.numBerths);
                IloArray<IloBoolVar> lambda_st(env, params.numBerths);
                IloArray<IloBoolVar> mu_st(env, params.numBerths);
                IloArray<IloNumVar> zeta_st(env, params.numBerths);
                IloArray<IloNumVar> eta_st(env, params.numBerths);
                
                for (int b = 0; b < params.numBerths; b++) {
                    // 生成变量名称
                    string omega_name = "omega_" + to_string(s) + "_" + to_string(t) + "_" + to_string(b);
                    string lambda_name = "lambda_" + to_string(s) + "_" + to_string(t) + "_" + to_string(b);
                    string mu_name = "mu_" + to_string(s) + "_" + to_string(t) + "_" + to_string(b);
                    
                    // 初始化变量并设置名称
                    omega_st[b] = IloBoolVar(env, omega_name.c_str());
                    lambda_st[b] = IloBoolVar(env, lambda_name.c_str());
                    mu_st[b] = IloBoolVar(env, mu_name.c_str());
                    
                    string zeta_name = "zeta_" + to_string(s) + "_" + to_string(t) + "_" + to_string(b);
                    string eta_name = "eta_" + to_string(s) + "_" + to_string(t) + "_" + to_string(b);
                    zeta_st[b] = IloNumVar(env, 0, params.planningHorizon * params.numShips, zeta_name.c_str());
                    eta_st[b] = IloNumVar(env, 0, params.planningHorizon * params.numShips, eta_name.c_str());
                }
                
                // 将t维度的数组添加到s维度数组
                omega_s[t] = omega_st;
                lambda_s[t] = lambda_st;
                mu_s[t] = mu_st;
                zeta_s[t] = zeta_st;
                eta_s[t] = eta_st;
            }
            
            // 将s维度的数组添加到全局数组
            omega.add(omega_s);
            lambda.add(lambda_s);
            mu.add(mu_s);
            zeta.add(zeta_s);
            eta.add(eta_s);
        }
        
        // 添加线性化约束(24-36)
        for (int s = 0; s < params.numShips; s++) {
            for (int t = 0; t < params.numShips; t++) {
                if (s == t) continue;
                for (int b = 0; b < params.numBerths; b++) {
                    // 约束(24): lambda + mu - omega = 0
                    model.add(lambda[s][t][b] + mu[s][t][b] - omega[s][t][b] == 0);
                    
                    // 约束(25-26): omega <= z_sb 和 omega <= z_tb
                    model.add(omega[s][t][b] <= z[s][b]);
                    model.add(omega[s][t][b] <= z[t][b]);
                    
                    // // 约束(27): omega >= z_sb + z_tb - 1
                    model.add(omega[s][t][b] >= z[s][b] + z[t][b] - 1);
                    // 约束(28-33): lambda和mu的线性化约束
                    // mu[s][t][b] = 1 表示s和t在同一泊位b，且s在t之后
                    model.add(mu[s][t][b] <= omega[s][t][b]);
                    model.add(mu[s][t][b] <= y[s][t]);
                    model.add(mu[s][t][b] >= omega[s][t][b] + y[s][t] - 1);
                    
                    // lambda[s][t][b] = 1 表示s和t在同一泊位b，且s在t之前
                    model.add(lambda[s][t][b] <= omega[s][t][b]);
                    model.add(lambda[s][t][b] <= 1 - y[s][t]); // 修正此处
                    model.add(lambda[s][t][b] >= omega[s][t][b] + (1 - y[s][t]) - 1);
                }
            }
        }
        
        // // 添加线性化约束(39-45)
        double M1 = params.planningHorizon + 150000; // 足够大的常数
        // for (int s = 0; s < params.numShips; s++) {
        //     for (int t = 0; t < params.numShips; t++) {
        //         if (s == t) continue;
        //         for (int b = 0; b < params.numBerths; b++) {
        //             // 约束(39): zeta + (M1 - gamma_s/p_sb)*omega - eta - M1*lambda >= 0

        //             // double gammaOverP = params.cargoWeight[s] / params.unloadingSpeed[s][b];
        //             double cargoWeight = params.cargoWeight[s];  
        //             double gammaOverP = 0;
        //             for(int k =0 ; k< params.numShipK;k++){
        //                 gammaOverP += cargoWeight / (params.numShipK * params.unloadingSpeed[s][b][k]);
        //             }
        //             model.add(zeta[s][t][b] + (M1 - gammaOverP) * omega[s][t][b] - 
        //                      eta[s][t][b] - M1 * lambda[s][t][b] >= 0);
                    
        //             // 约束(40-41): zeta和eta的范围约束
        //             model.add(e[t] + M1 * (omega[s][t][b] - 1) <= zeta[s][t][b]);
        //             model.add(zeta[s][t][b] <= e[t] + M1 * (1 - omega[s][t][b]));

        //             model.add(e[s] + M1 * (omega[s][t][b] - 1) <= eta[s][t][b]);
        //             model.add(eta[s][t][b] <= e[s] + M1 * (1 - omega[s][t][b]));
                    
        //             // 约束(42-43): zeta和eta的上界
        //             model.add(zeta[s][t][b] <= M1 * omega[s][t][b]);
        //             model.add(eta[s][t][b] <= M1 * omega[s][t][b]);

        //             // model.add(zeta[s][t][b] - eta[s][t][b] <= M1 * mu[s][t][b]);
        //             // model.add(eta[s][t][b] - zeta[s][t][b] <= M1 * (1 - mu[s][t][b]));
        //         }
        //     }
        // }

        // 关键修复：同一泊位上的不同船舶时间不重叠（基于 lambda/mu 的前后关系）
        // 若 lambda[s][t][b] = 1（s 在 t 之前，且两者都在泊位 b），则 e[s] + proc_s_b <= e[t]
        // 若 mu[s][t][b] = 1（t 在 s 之前，且两者都在泊位 b），则 e[t] + proc_t_b <= e[s]
        for (int s = 0; s < params.numShips; ++s) {
            for (int t = 0; t < params.numShips; ++t) {
                if (s == t) continue;
                for (int b = 0; b < params.numBerths; ++b) {
                    // 加工时长按被选泊位 b 计算（各舱串行卸货）
                    double proc_s_b = 0.0;
                    double proc_t_b = 0.0;
                    for (int k = 0; k < params.numShipK; ++k) {
                        double ps = params.unloadingSpeed[s][b][k];
                        if (ps <= 0) ps = 1.0; // 防止除零
                        proc_s_b += params.cargoWeight[s] / (params.numShipK * ps);
                        double pt = params.unloadingSpeed[t][b][k];
                        if (pt <= 0) pt = 1.0;
                        proc_t_b += params.cargoWeight[t] / (params.numShipK * pt);
                    }
                    // s 在 t 前：当 lambda=1 时收紧；否则放松到 M1
                    model.add(e[s] + proc_s_b <= e[t] + M1 * (1 - lambda[s][t][b]));
                    // t 在 s 前：当 mu=1 时收紧
                    model.add(e[t] + proc_t_b <= e[s] + M1 * (1 - mu[s][t][b]));
                }
            }
        }
        // 7. 求解模型
        IloCplex cplex(model);
        cout <<"导出模型"<<endl;
        // cout <<"导出模型"<<endl;
        // cplex.setOut(env.getNullStream()); // 关闭输出
        cplex.setParam(IloCplex::TiLim, 3600); // 设置时间限制为1小时
        
    // 计时：使用 CPLEX 的计时（与当前 ClockType 一致：CPU/WallClock/Deterministic）
    double t0 = cplex.getCplexTime();
    bool solved = cplex.solve();
    double solveSeconds = cplex.getCplexTime() - t0;

        if (solved) {
            env.out() << "模型求解成功！" << endl;
            env.out() << "目标函数值: " << cplex.getObjValue() << endl;
            env.out() << "求解时间(按当前计时方式): " << solveSeconds << " 秒" << endl;
            // 输出与最优解差距（MIP Gap）及最佳界
            try {
                double bestBound = cplex.getBestObjValue();
                double relGap = cplex.getMIPRelativeGap(); // 0 表示最优，>0 表示与最优的相对差距
                env.out() << "最佳界(best bound): " << bestBound << endl;
                env.out() << "与最优解差距(MIP gap): " << (relGap * 100.0) << " %" << endl;
            } catch (...) {
                // 某些情况下（非MIP或无效调用）可能抛异常，忽略即可
            }
            
            // 输出部分决策变量结果
            env.out() << "\n泊位分配结果(z_sb):" << endl;
            for (int s = 0; s < params.numShips; s++) {
                for (int b = 0; b < params.numBerths; b++) {
                    if (cplex.getValue(z[s][b]) > 0.5) {
                        env.out() << "船舶 " << s << " 分配到泊位 " << b << endl;
                    }
                }
            }
            
            // env.out() << "\n存储行分配结果(f_sr):" << endl;
            // for (int s = 0; s < params.numShips; s++) {
            //     for(int k =0 ; k< params.numShipK;k++){
            //         for (int r = 0; r < params.numRows; r++) {
            //             for( int v = 0; v < params.numSlotsPerRow; v++) {
            //                 if (cplex.getValue(x[s][k][r][v]) > 0.5) {
            //                     env.out() << "船舶 " << s <<" 货舱 " << k << " 存储在行 " << r << " 槽 " << v << endl;
            //                 }
            //             }
            //         }
            //     }
            // }

            // 输出每个舱占用的槽位区间（基于 f[s][k][r] 指定的行，合并连续的槽位为区间）
            env.out() << "\n每个货舱占用槽位区间:" << endl;
            for (int s = 0; s < params.numShips; ++s) {
                for (int k = 0; k < params.numShipK; ++k) {
                    int assignedRow = -1;
                    for (int r = 0; r < params.numRows; ++r) {
                        if (cplex.getValue(f[s][k][r]) > 0.5) {
                            assignedRow = r;
                            break;
                        }
                    }
                    if (assignedRow == -1) continue; // 未分配行

                    std::vector<int> occ;
                    for (int v = 0; v < params.numSlotsPerRow; ++v) {
                        if (cplex.getValue(x[s][k][assignedRow][v]) > 0.5) occ.push_back(v);
                    }
                    if (occ.empty()) continue;

                    // 合并连续区间
                    std::vector<std::pair<int,int>> intervals;
                    int start = occ[0], prev = occ[0];
                    for (size_t idx = 1; idx < occ.size(); ++idx) {
                        int cur = occ[idx];
                        if (cur == prev + 1) {
                            prev = cur;
                        } else {
                            intervals.emplace_back(start, prev);
                            start = cur; prev = cur;
                        }
                    }
                    intervals.emplace_back(start, prev);

                    // 打印结果
                    env.out() << "船舶 " << s << " 货舱 " << k << " 行 " << assignedRow << ": ";
                    for (size_t ii = 0; ii < intervals.size(); ++ii) {
                        auto pr = intervals[ii];
                        if (pr.first == pr.second) env.out() << "[" << pr.first << "]";
                        else env.out() << "[" << pr.first << "-" << pr.second << "]";
                        if (ii + 1 < intervals.size()) env.out() << ", ";
                    }
                    env.out() << endl;
                }
            }
            
            env.out() << "\n卸载开始时间(e_s):" << endl;
            for (int s = 0; s < params.numShips; s++) {
                env.out() << "船舶 " << s << ": " << cplex.getValue(e[s]) << " 小时" << endl;
            }
            env.out() << "\n卸载开始时间(e_sk):" << endl;
            for (int s = 0; s < params.numShips; s++) {
                for(int k =0 ; k< params.numShipK;k++){
                    env.out() << "船舶 " << s <<" 货舱 " << k << ": " << cplex.getValue(e_sk[s][k]) << " 小时" << endl;
                }
            }

            // 每艘船：转运成本、存储成本、靠泊时间 分解与汇总
            try {
                env.out() << "\n每艘船的成本与时间分解:" << endl;
                double totalTrans = 0.0, totalStore = 0.0, totalBerthTime = 0.0;

                // 也写入 CSV
                if (!mkdir_p(OUTPUT_DIR)) {
                    throw std::runtime_error("无法创建输出目录: " + OUTPUT_DIR);
                }
                std::ofstream obcsv(OUTPUT_DIR + "/objective_breakdown.csv");
                obcsv << "ship,transshipment_cost,storage_cost,berth_time,alpha,beta,weighted_contribution\n";

                for (int s = 0; s < params.numShips; ++s) {
                    // 1) 转运成本（按当前模型定义：x 与 z 的乘积）
                    double transCostS = 0.0;
                    for (int k = 0; k < params.numShipK; ++k) {
                        int req = params.requiredSlots[s][k];
                        if (req <= 0) continue; // 防止除零
                        double perSlotWeight = params.cargoWeight[s] / (static_cast<double>(req) * params.numShipK);
                        for (int b = 0; b < params.numBerths; ++b) {
                            double zval = cplex.getValue(z[s][b]);
                            if (zval <= 1e-6) continue; // 只用被选泊位的贡献（其它泊位 z≈0）
                            for (int r = 0; r < params.numRows; ++r) {
                                for (int v = 0; v < params.numSlotsPerRow; ++v) {
                                    double xval = cplex.getValue(x[s][k][r][v]);
                                    if (xval <= 1e-6) continue;
                                    transCostS += params.transshipmentCost[b][r][v] * perSlotWeight * xval * zval;
                                }
                            }
                        }
                    }

                    // 2) 存储成本（按行与槽，和 x）
                    double storeCostS = 0.0;
                    for (int k = 0; k < params.numShipK; ++k) {
                        for (int r = 0; r < params.numRows; ++r) {
                            for (int v = 0; v < params.numSlotsPerRow; ++v) {
                                double xval = cplex.getValue(x[s][k][r][v]);
                                if (xval <= 1e-6) continue;
                                storeCostS += params.storageCost[s][k][r] * xval;
                            }
                        }
                    }

                    // 3) 靠泊时间（等待 + 卸货时长，按选泊位的速度）
                    double berthTimeS = 0.0;
                    double es = cplex.getValue(e[s]);
                    berthTimeS += (es - params.arrivalTime[s]);
                    for (int b = 0; b < params.numBerths; ++b) {
                        double zval = cplex.getValue(z[s][b]);
                        if (zval <= 1e-6) continue;
                        for (int k = 0; k < params.numShipK; ++k) {
                            double speed = params.unloadingSpeed[s][b][k];
                            if (speed <= 0) speed = 1.0; // 防止除零
                            berthTimeS += params.cargoWeight[s] / (speed * params.numShipK) * zval;
                        }
                    }

                    totalTrans += transCostS;
                    totalStore += storeCostS;
                    totalBerthTime += berthTimeS;

                    env.out() << "船舶 " << s
                              << " | 转运成本: " << transCostS
                              << " | 存储成本: " << storeCostS
                              << " | 靠泊时间: " << berthTimeS << " 小时" << endl;

                    double weighted = params.alpha * (transCostS + storeCostS) + params.beta * berthTimeS;
                    obcsv << s << "," << transCostS << "," << storeCostS << "," << berthTimeS
                          << "," << params.alpha << "," << params.beta << "," << weighted << "\n";
                }

                env.out() << "\n合计 | 转运成本: " << totalTrans
                          << " | 存储成本: " << totalStore
                          << " | 靠泊时间: " << totalBerthTime << " 小时" << endl;

                obcsv.close();
            } catch (std::exception &ex) {
                env.out() << "打印/写入成本分解时出错: " << ex.what() << std::endl;
            }

            // 写入结果到 output 目录（CSV 格式，可由 Excel 打开）
            try {
                // 确保输出目录存在
                if (!mkdir_p(OUTPUT_DIR)) {
                    throw std::runtime_error("无法创建输出目录: " + OUTPUT_DIR);
                }

                // 泊位分配
                {
                    std::ofstream ofs(OUTPUT_DIR + "/berth_assignment.csv");
                    ofs << "ship,berth\n";
                    for (int s = 0; s < params.numShips; s++) {
                        for (int b = 0; b < params.numBerths; b++) {
                            if (cplex.getValue(z[s][b]) > 0.5) ofs << s << "," << b << "\n";
                        }
                    }
                }

                // 每个槽的分配 (s,k,row,slot)
                {
                    std::ofstream ofs(OUTPUT_DIR + "/slot_allocations.csv");
                    ofs << "ship,k,row,slot\n";
                    for (int s = 0; s < params.numShips; ++s) {
                        for (int k = 0; k < params.numShipK; ++k) {
                            for (int r = 0; r < params.numRows; ++r) {
                                for (int v = 0; v < params.numSlotsPerRow; ++v) {
                                    if (cplex.getValue(x[s][k][r][v]) > 0.5) ofs << s << "," << k << "," << r << "," << v << "\n";
                                }
                            }
                        }
                    }
                }

                // 每个货舱占用区间
                {
                    std::ofstream ofs(OUTPUT_DIR + "/intervals.csv");
                    ofs << "ship,k,row,intervals\n";
                    for (int s = 0; s < params.numShips; ++s) {
                        for (int k = 0; k < params.numShipK; ++k) {
                            int assignedRow = -1;
                            for (int r = 0; r < params.numRows; ++r) {
                                if (cplex.getValue(f[s][k][r]) > 0.5) { assignedRow = r; break; }
                            }
                            if (assignedRow == -1) continue;

                            std::vector<int> occ;
                            for (int v = 0; v < params.numSlotsPerRow; ++v) {
                                if (cplex.getValue(x[s][k][assignedRow][v]) > 0.5) occ.push_back(v);
                            }
                            if (occ.empty()) continue;

                            // 合并连续区间为字符串
                            std::ostringstream oss;
                            int start = occ[0], prev = occ[0];
                            for (size_t idx = 1; idx < occ.size(); ++idx) {
                                int cur = occ[idx];
                                if (cur == prev + 1) { prev = cur; }
                                else {
                                    if (start == prev) oss << start;
                                    else oss << start << "-" << prev;
                                    oss << ";";
                                    start = cur; prev = cur;
                                }
                            }
                            if (start == prev) oss << start; else oss << start << "-" << prev;

                            ofs << s << "," << k << "," << assignedRow << "," << '"' << oss.str() << '"' << "\n";
                        }
                    }
                }

                // e_s
                {
                    std::ofstream ofs(OUTPUT_DIR + "/e_s.csv");
                    ofs << "ship,e_s\n";
                    for (int s = 0; s < params.numShips; ++s) ofs << s << "," << cplex.getValue(e[s]) << "\n";
                }

                // e_sk
                {
                    std::ofstream ofs(OUTPUT_DIR + "/e_sk.csv");
                    ofs << "ship,k,e_sk\n";
                    for (int s = 0; s < params.numShips; ++s) {
                        for (int k = 0; k < params.numShipK; ++k) {
                            ofs << s << "," << k << "," << cplex.getValue(e_sk[s][k]) << "\n";
                        }
                    }
                }

            } catch (std::exception &ex) {
                env.out() << "写输出文件时出错: " << ex.what() << std::endl;
            }

        } else {
            
            env.out() << "求解状态: " << cplex.getStatus() << endl;
            env.out() << "求解时间(按当前计时方式): " << solveSeconds << " 秒" << endl;
            // 若有可用的界与gap，尽量输出用于诊断
            try {
                double bestBound = cplex.getBestObjValue();
                env.out() << "最佳界(best bound): " << bestBound << endl;
                double relGap = cplex.getMIPRelativeGap();
                env.out() << "与最优解差距(MIP gap): " << (relGap * 100.0) << " %" << endl;
            } catch (...) {
                // 忽略
            }
            env.out() << "不可行解分析:" << endl;
            cplex.exportModel("infeasible_model.lp");
            // 尝试找到导致不可行的关键约束
            IloNumVarArray vars(env);
            IloRangeArray ranges(env);
            // cplex.getInfeasible(vars, ranges);
            env.out() << "不可行变量数: " << vars.getSize() << endl;
            env.out() << "不可行约束数: " << ranges.getSize() << endl;

        }
        
        // 8. 释放资源
        cplex.end();
        model.end();
        env.end();
        
        return 0;
    } catch (IloException& e) {
        cerr << "CPLEX异常: " << e << endl;
    } catch (...) {
        cerr << "未知异常" << endl;
    }
    
    return 1;
}