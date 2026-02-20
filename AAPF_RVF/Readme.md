## ファイルリスト
- params.m
- runner.m
- PreProcess_Simulation.m
- ExeSimulation.m
- ApproachSim.m
- Aapf.m
- Rvf.m
- EOM.m
- AccelerationConstraint.m
- environments.m
- target.m
- target_KOZ1.m
- utility.m
- PreProcess_visualization

### params.m
シミュレーションの基本的な設定を記述するファイル．</br>
シミュレーションの設定ごとにサブディレクトリに格納することを想定している．

### runner.m
シミュレーションを実行するためのファイル．</br>
どの設定で実行するかを選択して，実行するとパラメーターの読み込みからシミュレーション結果の保存まで一括で実行される．

### PreProcess_Simulation.m
シミュレーション実行のための前処理を記述した．
まず，シミュレーション期間全体のターゲットの運動を事前計算する．
ターゲットの軌道はケプラーの軌道要素で`target.m`に定義されており，ECI座標系で軌道の時系列データを計算する．
さらに，ターゲットの姿勢運動をECI座標系を基準として計算する．
その後，軌道データと姿勢運動のデータから，Hill座標系を基準としたターゲットの姿勢に変換する．
また，シミュレーションデータを出力するための配列の確保も行う．

### ExeSimulation.m
ApproachSimクラスに設定を渡して，ApproachSim::run()によって出力されるシミュレーション結果を保存する．
`foreach`で並列処理することを想定しているが，デバックの際に`for`に変更してからそのまま放置されている．

### ApproachSim.m
ApproachSimクラスを定義する．
ApproachSimクラスは1ケースのシミュレーションを処理するためのクラスである．
シミュレーションを実行する`run()`関数以外に，推力制約や終端条件，航法誤差を扱うメンバ関数がある．

### Aapf.m
引力ポテンシャルの適応更新則を定義したクラス．</br>
`Aapf::update()`を実行することにより更新された引力ポテンシャル場の形状行列を得ることができる．

### Rvf.m
`Aapf.m`と同様．RVF向けのクラス．

### EOM.m
ヒル方程式を使った相対運動の状態方程式

### AccelerationConstraint.m
加速度制約を定義した．
フェーズの切り替わりの閾値を`AccelaretionConstraint::maximumRadiusToSync()`で取得できる．
`AccelaretionConstraint::rvfReferenceConstraint()`でフェーズ２におけるRVFの参照角速度の上限を得る．

### environments.m
重力定数が定義されている．J2摂動などを考慮したい場合はこのファイルを編集する．

### target.m
environmentsを継承したtarghetクラスを定義する．
ターゲットの状態方程式や安全領域を定義する．
ここでの安全領域は楕円を複数接続して構成されるもの．

### target_KOZ1.m
targetクラスを継承した．
安全領域を修論で述べたものに変更した．

### utility.m
便利そうな関数をまとめた．

### PreProcess_visualization
結果を可視化する際のプリプロセス．
全体のシミュレーション結果からj番目の条件の結果を取り出すことができる．

## 実行方法
1. シミュレーション設定を記述した`params.m`を用意する．
2. `runner.m`の`directory`で`params.m`のあるディレクトリを指定する．
3. `runner.m`を実行する
4. シミュレーション結果が`MMddHHmmss.mat`に出力される
記載事項は`params_example.m`を参照するしてください．

## 結果の可視化
修論で使用したのもは各条件のところにまとめておいてある．
ファイル名からどんな結果を出力するかは類推できると思うので割愛する．
一部の可視化用ファイルは
`PreProcess_visualization`を事前に実行することを前提としていることがある．
