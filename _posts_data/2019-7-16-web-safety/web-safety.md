# Web 安全
## 安全世界观
### 安全的本质：信任
1. 划分信任域
2. 安全方案需要信任某些前提条件
### 安全是一个持续的过程
1. 没有银弹， 攻击技术和防御技术在碰撞中不断发展
### 安全的三要素 CIA
1. 机密性 Confidentiality
2. 完整性 Integrity
3. 可用性 Availability
### 安全评估
1. 资产等级划分
2. 威胁分析
3. 风险分析
4. 确认解决方案
### 安全方案原则
#### Security by default
1. 白名单>黑名单
2. 最小权限原则
#### 纵深防御原则
1. 不同层面实施安全方案
2. 在正确的地方做正确的事情
#### 数据与代码分离原则
#### 不可预测性原则
## 客户端脚本安全
### 浏览器安全
#### 同源策略
##### 协议，域名，端口相同
### 跨站脚本攻击XSS
#### 简介
##### “HTML注入”,插入恶意脚本。跨不跨域并不重要
#### 类型
1. 反射型
通过用户输入攻击，通常引诱用户点击恶意链接
2. 存储型
持久化存储在服务器
#### 危害
1. Cookie劫持
2. 钓鱼
3. 构造请求
4. 识别浏览器，软件，IP，浏览记录等信息
#### 防御
1. HttpOnly防止Cookie劫持
2. 输入检查，过滤特殊字符
3. 输出检查，编码转义
针对不同类型的文本和不同情景，需要用不同的转义方式
### 跨站请求伪造CSRF
#### 简介
##### 伪造跨域请求，执行非用户本意的操作
#### 本质
##### 攻击者能猜测请求的所有参数
#### 防御
1. 一致的做法：在请求上添加攻击者不能预测的token
2. 验证码
3. Referer Check
### 点击劫持
#### 简介
##### 视觉欺骗，利用不可见iframe覆盖在网页上，诱使用户点击
#### 防御
1. frame busting. 写js代码，防止iframe 嵌套
2. X-Frame-Options/Content Swecurity Policy
## 服务器端应用安全
### 注入攻击
#### 简介
##### 将用户输入数据当作代码执行
#### 条件
1. 用户可以控制输入
2. 程序要执行的程序拼接了用户代码
#### SQL注入防御
1. 使用预编译语句绑定变量
2. 使用安全的存储过程
3. 检查数据类型
4. 使用安全的编码函数
#### 安全原则
##### 数据和代码分离，在拼接处做安全检查
### 认证与会话管理
#### 简介
##### 认证的目的是识别用户，实际上是验证凭证的过程。
系统---屋子
认证---钥匙开锁
钥匙---凭证
#### 密码
1. 成本低，过程简单
2. 没有统一策略，应对复杂度有要求，不要使用用户公开数据
2. 应以不可逆方式加密保存
3. 加Salt防止彩虹表
#### 多因素认证
1. 动态口令
2. 数字证书
3. 手机绑定
4. .......
#### Session
##### 简介
###### 认证后获得的对用户透明的凭证
##### Session Fixation攻击
###### 用户使用攻击者的SessionId登陆，服务器使用了该SessionId
1. 用户登陆后，重写SessionId
##### Session 保持攻击
###### 通过不停发送请求，保持Session存活
1. 定期销毁Session
#### 单点登陆
##### 简介
###### 登陆一次，访问所有系统
方便，风险集中
##### 系统
###### OpenID
### 访问控制
#### 简介
##### 授权，即主体对客体操作的限制
#### 垂直权限管理
##### 基于用户权限的访问控制（Role-Based Access Control）
1. 用户---角色---权限
2. 角色之间有权限高低之分
#### 水平权限管理
##### 简介
1. 对同一角色的不同用户进行权限管理
2. 基于数据的访问控制
##### 方案
1. 用户组
2. 规则引擎
#### 系统
1. OAuth
#### 法则
1. 最小权限原则
### 分布式拒绝服务攻击DDOS
#### 本质
1. 有限资源的无限滥用
1. 防御：根据攻击特征，清洗流量
#### SYN flood
1. 发送SYN包后不回应SYN/ACK
2. 统计IP访问频率，清洗流量
#### 应用层DDOS防御
##### 性能优化
##### 网络架构分流：负载均衡，CDN
##### 限制请求频率
1. 验证码
2. 让客户端解析JS代码
3. 服务器配置
4. IP和Cookie信息