/**
 * @file Referee.hpp
 * @author Xu Zihan (im.xuzihan@outlook.com)
 * @brief Referee I/O logic
 * @version 0.1
 * @date 2020-03-19
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef OSSIAN_REFEREE_HPP
#define OSSIAN_REFEREE_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/UART.hpp>
#include <ossian/MultiThread.hpp>
#include <ossian/IOData.hpp>
#include <spdlog/spdlog.h>

#include <mutex>

#include "DJICRCHelper.hpp"

#pragma pack(push,1)

/**
 * @brief 连命令码的帧头结构
 */
struct FrameHeaderWithCmd
{
	uint8_t m_SOF;         ///< 起始字节(0xA5)
	uint16_t m_DataLength; ///< 数据长度
	uint8_t m_Seq;         ///< 包序号
	uint8_t m_CRC8;        ///< 帧头CRC8校验
	uint16_t m_CmdId;      ///< 命令码
};

/**
 * @brief 帧尾
 */
struct FrameTail
{
	uint16_t m_CRC16; ///< 整包校验
};

/**
 * @brief 比赛状态数据，1Hz 周期发送(0x0001)
 */
struct MatchStatus
{
	static constexpr uint16_t CMD_ID = 0x0001;
	static constexpr size_t LENGTH   = 3;

	uint8_t m_GameType : 4;     ///< 比赛类型：1:机甲大师赛 2:单项赛 3:人工智能挑战赛
	uint8_t m_GameProgress : 4; ///< 当前比赛阶段：0:未开始比赛 1:准备阶段 2:自检阶段 3:5s倒计时 4:对战中 5:比赛结算中
	uint16_t m_StageRemainTime; ///< 当前阶段剩余时间(s)
};

/**
 * @brief 比赛结果数据，比赛结束后发送(0x0002)
 */
struct MatchResult
{
	static constexpr uint16_t CMD_ID = 0x0002;
	static constexpr size_t LENGTH   = 1;

	uint8_t m_Winner; ///< 比赛结果：0:平局 1:红方胜利 2:蓝方胜利
};

/**
 * @brief 比赛机器人血量数据，1Hz 周期发送(0x0003)
 */
struct RobotHP
{
	static constexpr uint16_t CMD_ID = 0x0003;
	static constexpr size_t LENGTH   = 32;

	uint16_t m_Red1HP;       ///< 红1英雄
	uint16_t m_Red2HP;       ///< 红2工程
	uint16_t m_Red3HP;       ///< 红3步兵
	uint16_t m_Red4HP;       ///< 红4步兵
	uint16_t m_Red5HP;       ///< 红5步兵
	uint16_t m_Red7HP;       ///< 红7哨兵
	uint16_t m_RedOutpostHP; ///< 红前哨站
	uint16_t m_RedBaseHP;    ///< 红基地

	uint16_t m_Blue1HP;       ///< 蓝1英雄
	uint16_t m_Blue2HP;       ///< 蓝2工程
	uint16_t m_Blue3HP;       ///< 蓝3步兵
	uint16_t m_Blue4HP;       ///< 蓝4步兵
	uint16_t m_Blue5HP;       ///< 蓝5步兵
	uint16_t m_Blue7HP;       ///< 蓝7哨兵
	uint16_t m_BlueOutpostHP; ///< 蓝前哨站
	uint16_t m_BlueBaseHP;    ///< 蓝基地
};

/**
 * @brief 场地事件数据，事件改变后发送(0x0101)
 */
struct VenueEvent
{
	static constexpr uint16_t CMD_ID = 0x0101;
	static constexpr size_t LENGTH   = 4;

	uint8_t m_TarmacOccupation : 2; ///< 己方停机坪占领状态：0 为无机器人占领；1 为空中机器人已占领但未停桨；2 为空中机器人已占领并停桨

	bool m_HealingOccupation1 : 1; ///< 己方补给站 1 号补血点占领状态，1 为已占领
	bool m_HealingOccupation2 : 1; ///< 己方补给站 2 号补血点占领状态，1 为已占领
	bool m_HealingOccupation3 : 1; ///< 己方补给站 3 号补血点占领状态，1 为已占领

	bool m_StrikePointOccupation : 1; ///< 为打击点占领状态，1 为占领
	bool m_SmallEnergyActivation : 1; ///< 为小能量机关激活状态，1 为已激活
	bool m_BigEnergyActivation : 1;   ///< 为大能量机关激活状态，1 为已激活

	bool m_PassOccupation : 1;           ///< 己方关口占领状态，1 为已占领
	bool m_BunkerOccupation : 1;         ///< 己方碉堡占领状态，1 为已占领
	bool m_ResourceIslandOccupation : 1; ///< 己方资源岛占领状态，1 为已占领

	bool m_BaseShieldStatus : 1;   ///< 己方基地护盾状态：1 为基地有虚拟护盾血量；0 为基地无虚拟护盾血量
	uint8_t m_Reserve0 : 4;        ///< 保留
	uint8_t m_Reserve1 : 8;        ///< 保留
	uint8_t m_Reserve2 : 4;        ///< 保留
	uint8_t m_RedDefenseBuff : 2;  ///< ICRA 红方防御加成：0：防御加成未激活；1：防御加成 5s 触发激活中；2：防御加成已激活
	uint8_t m_BlueDefenseBuff : 2; ///< ICRA 蓝方防御加成：0：防御加成未激活；1：防御加成 5s 触发激活中；2：防御加成已激活
};

/**
 * @brief 场地补给站动作标识数据，动作改变后发送(0x0102)
 */
struct SupplyStation
{
	static constexpr uint16_t CMD_ID = 0x0102;
	static constexpr size_t LENGTH   = 3;

	uint8_t m_SupplyProjectileId; ///< 补给站口ID：1:1号补给口 2:2号补给口

	/**
	 * @brief 补弹机器人 ID
	 * 0 为当前无机器人补弹
	 * 1 为红方英雄机器人补弹
	 * 2 为红方工程机器人补弹
	 * 3/4/5 为红方步兵机器人补弹
	 * 11 为蓝方英雄机器人补弹
	 * 12 为蓝方工程机器人补弹
	 * 13/14/15 为蓝方步兵机器人补弹
	 */
	uint8_t m_SupplyRobotId;

	uint8_t m_SupplyProjectileStep; ///< 出弹口开闭状态：0 为关闭，1 为弹丸准备中，2 为弹丸下落

	uint8_t m_SupplyProjectileNum; ///< 补弹数量： 50：50 颗弹丸；100：100 颗弹丸；150：150 颗弹丸；200：200 颗弹丸。 
};

/**
 * @brief 请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）(0x103)
 */
struct SupplyStationQuery
{
	static constexpr uint16_t CMD_ID = 0x0103;
	static constexpr size_t LENGTH   = 2;

	uint8_t m_SupplyProjectileId; ///< 补给站补弹口ID：1:1号补给口

	/**
	 * @brief 补弹机器人 ID
	 * 1 为红方英雄机器人补弹
	 * 2 为红方工程机器人补弹，
	 * 3/4/5 为红方步兵机器人补弹
	 * 11 为蓝方英雄机器人补弹
	 * 12 为蓝方工程机器人补弹
	 * 13/14/15 为蓝方步兵机器人补弹
	 */
	uint8_t m_SupplyRobotId;

	uint8_t m_SupplyNum; ///< 补弹数目：50:请求50颗弹丸下落
};

/**
 * @brief 裁判警告数据，警告发生后发送(0x0104)
 */
struct RefereeWarning
{
	static constexpr uint16_t CMD_ID = 0x0104;
	static constexpr size_t LENGTH   = 2;

	uint8_t m_Level;       ///< 警告等级
	uint8_t m_FoulRobotId; ///< 犯规机器人ID：1级以及5级警告时，机器人ID为0，二三四级警告时，机器人ID为犯规机器人ID
};

/**
 * @brief 机器人状态数据，10Hz 周期发送(0x0201)
 */
struct RobotStatus
{
	static constexpr uint16_t CMD_ID = 0x0201;
	static constexpr size_t LENGTH   = 27;

	/**
	* @brief 机器人 ID：
	* 本机器人 ID：
	* 1：红方英雄机器人；
	* 2：红方工程机器人；
	* 3/4/5：红方步兵机器人；
	* 6：红方空中机器人；
	* 7：红方哨兵机器人；
	* 8：红方飞镖机器人；
	* 9：红方雷达站；
	* 101：蓝方英雄机器人；
	* 102：蓝方工程机器人；
	* 103/104/105：蓝方步兵机器人；
	* 106：蓝方空中机器人；
	* 107：蓝方哨兵机器人；
	* 108：蓝方飞镖机器人；
	* 109：蓝方雷达站
	 */

	uint8_t m_RobotId;
	uint8_t m_RobotLevel;                  ///< 机器人等级 1：一级；2：二级；3：三级
	uint16_t m_RemainHp;                   ///< 机器人剩余血量
	uint16_t m_MaxHp;                      ///< 机器人上限血量
	uint16_t m_Shooter17Id1CoolingRate;    ///< 机器人 1 号 17mm 枪口每秒冷却值
	uint16_t m_Shooter17Id1HeatLimit;      ///< 机器人 1 号 17mm 枪口热量上限
	uint16_t m_Shooter17Id1SpeedLimit;     ///< 机器人 1 号 17mm 枪口上限速度 单位 m/s
	uint16_t m_Shooter17Id2CoolingRate;    ///< 机器人 2 号 17mm 枪口每秒冷却值
	uint16_t m_Shooter17Id2HeatLimit;      ///< 机器人 2 号 17mm 枪口热量上限
	uint16_t m_Shooter17Id2SpeedLimit;     ///< 机器人 2 号 17mm 枪口上限速度 单位 m/s
	uint16_t m_Shooter42Id1CoolingRate;    ///< 机器人 42mm 枪口每秒冷却值
	uint16_t m_Shooter42Id1HeatLimit;      ///< 机器人 42mm 枪口热量上限
	uint16_t m_Shooter42Id1SpeedLimit;     ///< 机器人 42mm 枪口上限速度 单位 m/s
	uint16_t m_ChassisPowerLimit;          ///< 机器人底盘功率上限
	uint8_t m_MainsPowerGimbalOutput : 1;  ///< 主控电源输出情况：0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
	uint8_t m_MainsPowerChassisOutput : 1; ///< 主控电源输出情况：1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出； 
	uint8_t m_MainsPowerShooterOutput : 1; ///< 主控电源输出情况：2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
};


/**
 * @brief 实时功率热量数据，50Hz 周期发送(0x0202)
 */
struct PowerHeatData
{
	static constexpr uint16_t CMD_ID = 0x0202;
	static constexpr size_t LENGTH   = 16;

	uint16_t m_ChassisVolt;        ///< 底盘输出电压 单位 毫伏 
	uint16_t m_ChassisCurrent;     ///< 底盘输出电流 单位 毫安 
	float m_ChassisPower;          ///< （四字节）底盘输出功率 单位 W 瓦
	uint16_t m_ChassisPowerBuffer; ///< 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J 
	uint16_t m_Shooter17Id1Heat;   ///< 1号 17mm 枪口热量 
	uint16_t m_Shooter17Id2Heat;   ///< 2号 17mm 枪口热量 
	uint16_t m_Shooter42Id1Heat;   ///< 42mm 枪口热量 
};

/**
 * @brief 机器人位置数据，10Hz 发送(0x0203)
 */
struct RobotPosition
{
	static constexpr uint16_t CMD_ID = 0x0203;
	static constexpr size_t LENGTH   = 16;

	float m_X;   ///< （四字节）位置 x 坐标，单位 m 
	float m_Y;   ///< （四字节）位置 y 坐标，单位 m 
	float m_Z;   ///< （四字节）位置 z 坐标，单位 m 
	float m_Yaw; ///< （四字节）位置枪口，单位度
};

/**
 * @brief 机器人增益数据，增益状态改变后发送(0x0204)
 */
struct RobotRuneBuff
{
	static constexpr uint16_t CMD_ID = 0x0204;
	static constexpr size_t LENGTH   = 1;

	bool m_HealingStatus : 1;             ///< 机器人血量补血状态
	bool m_BarrelCoolingAcceleration : 1; ///< 枪口热量冷却加速
	bool m_DefenseBuff : 1;               ///< 机器人防御加成
	bool m_AttackBuff : 1;                ///< 机器人攻击加成
	uint8_t m_Reserve : 4;                ///< 保留
};

/**
 * @brief 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送(0x0205)
 */
struct AerialRobotStatus
{
	static constexpr uint16_t CMD_ID = 0x0205;
	static constexpr size_t LENGTH   = 3;

	uint8_t m_EnergyPoint; ///< 积累的能量点
	uint8_t m_AttackTime;  ///< 可攻击时间 单位 s。30s 递减至 0
};

/**
 * @brief 伤害状态数据，伤害发生后发送(0x0206)
 */
struct DamageStatus
{
	static constexpr uint16_t CMD_ID = 0x0206;
	static constexpr size_t LENGTH   = 1;

	/**
	 * @brief 装甲ID
	 * 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为0-4号代表机器人的五个装甲片
	 * 其他血量变化类型，该变量数值为 0
	 */
	uint8_t m_ArmorId : 4;

	/**
	 * @brief 血量变化类型
	 * 0x0 装甲伤害扣血；
	 * 0x1 模块掉线扣血；
	 * 0x2 超射速扣血；
	 * 0x3 超枪口热量扣血；
	 * 0x4 超底盘功率扣血；
	 * 0x5 装甲撞击扣血
	 */
	uint8_t m_HurtType : 4;
};

/**
 * @brief 实时射击数据，弹丸发射后发送(0x0207)
 */
struct ShootData
{
	static constexpr uint16_t CMD_ID = 0x0207;
	static constexpr size_t LENGTH   = 7;

	uint8_t m_BulletType; ///< 弹丸类型: 1：17mm 弹丸 2：42mm 弹丸 
	uint8_t m_ShooterId;  ///< 发射机构 ID：1：1 号 17mm 发射机构  2：2 号 17mm 发射机构  3：42mm 发射机构
	uint8_t m_BulletFreq; ///< 弹丸射频 单位 Hz
	float m_BulletSpeed;  ///< （四字节）弹丸射速 单位 m/s 
};

/**
 * @brief 弹丸剩余发射数，仅空中机器人以及哨兵机器人主控发送该数据，1Hz 周期发送(0x0208)
 */
struct BulletRemain
{
	static constexpr uint16_t CMD_ID = 0x0208;
	static constexpr size_t LENGTH   = 6;

	uint16_t m_BulletRemainingNum17mm; ///< 17mm 子弹剩余发射数目
	uint16_t m_BulletRemainingNum42mm; ///< 42mm 子弹剩余发射数目
	uint16_t m_CoinRemainingNum;       ///< 剩余金币数量
};

/**
 * @brief 机器人RFID状态，发送范围：单一机器人。1Hz 周期发送(0x0209)
 */
struct RFIDStatus
{
	static constexpr uint16_t CMD_ID = 0x0209;
	static constexpr size_t LENGTH   = 4;

	bool m_RFIDBaseBuff : 1;     ///< 基地增益点RFID状态标志位
	bool m_RFIDHighlandBuff : 1; ///< 高地增益点RFID状态标志位
	bool m_RFIDWindmillBuff : 1; ///< 能量机关激活点RFID状态标志位
	bool m_RFIDSlopeBuff : 1;    ///< 飞坡增益点RFID状态标志位
	bool m_RFIDSentryBuff : 1;   ///< 前哨岗增益点RFID状态标志位
	bool m_RFIDResourceBuff : 1; ///< 资源岛增益点RFID状态标志位
	bool m_RFIDHealingBuff : 1;  ///< 补血点增益点RFID状态标志位
	bool m_RFIDEngineerBuff : 1; ///< 工程机器人补血卡RFID状态标志位
	uint8_t m_Reserve0;          ///< bit8-15: 保留
	uint8_t m_Reserve1;          ///< bit16-23: 保留
	uint8_t m_Reserve2 : 2;      ///< bit24-25: 保留
	bool m_RFIDF1 : 1;           ///< 人工智能挑战赛RFID状态 F1
	bool m_RFIDF2 : 1;           ///< 人工智能挑战赛RFID状态 F2
	bool m_RFIDF3 : 1;           ///< 人工智能挑战赛RFID状态 F3
	bool m_RFIDF4 : 1;           ///< 人工智能挑战赛RFID状态 F4
	bool m_RFIDF5 : 1;           ///< 人工智能挑战赛RFID状态 F5
	bool m_RFIDF6 : 1;           ///< 人工智能挑战赛RFID状态 F6
};

// ====================================================================
// 机器人间交互数据 开始(0x0301)
// ====================================================================

/**
 * @brief 机器人间交互数据头
 */
struct InteractiveHeader
{
	static constexpr size_t LENGTH = 6;

	uint16_t m_DataCmdId;  ///< 数据段的内容ID
	uint16_t m_SenderID;   ///< 发送者ID
	uint16_t m_ReceiverID; ///< 接收者ID
};

/**
 * @brief 客户端删除图形
 */
struct GraphicDelete
{
	static constexpr size_t LENGTH = 2;

	uint8_t m_OperateType; ///< 图形操作: 0: 空操作 1: 删除图层 2: 删除所有
	uint8_t m_Layer;       ///< 图层数：0-9
};

/**
 * @brief 图形数据，用于绘制图形以及配置字符选项
 */
struct GraphicData
{
	static constexpr size_t LENGTH = 15;

	uint8_t m_GraphicName[3];   ///< 图形名: 在删除，修改等操作中，作为客户端的索引
	uint32_t m_OperateType : 3; ///< 图形操作: 0: 空操作 1: 增加 2: 修改 3: 删除
	uint32_t m_GraphicType : 3;
	uint32_t m_Layer : 4;
	uint32_t m_Color : 4;
	uint32_t m_StartAngle : 9;
	uint32_t m_EndAngle : 9;
	uint32_t m_Width : 10;
	uint32_t m_StartX : 11;
	uint32_t m_StartY : 11;
	uint32_t m_Radius : 10;
	uint32_t m_EndX : 11;
	uint32_t m_EndY : 11;
};

/**
 * @brief 删除图形选项
 */
struct DeleteClientGraphic
{
	static constexpr uint16_t CMD_ID      = 0x0301;
	static constexpr uint16_t DATA_CMD_ID = 0x0100;
	static constexpr size_t LENGTH        = InteractiveHeader::LENGTH + GraphicDelete::LENGTH;

	InteractiveHeader m_Header;
	GraphicDelete m_DeleteOption;
};


template <size_t N>
struct ConvertNToCmdIdOffset
{
};

template <>
struct ConvertNToCmdIdOffset<1>
{
	static constexpr std::size_t value = 0;
};

template <>
struct ConvertNToCmdIdOffset<2>
{
	static constexpr std::size_t value = 1;
};

template <>
struct ConvertNToCmdIdOffset<5>
{
	static constexpr std::size_t value = 2;
};

template <>
struct ConvertNToCmdIdOffset<7>
{
	static constexpr std::size_t value = 3;
};

/**
 * @brief 增加/修改图形
 * @tparam N 修改图形的个数，取值范围: {1, 2, 5, 7}
 */
template <size_t N>
struct ClientGraphicShapeModify
{
	static constexpr uint16_t CMD_ID      = 0x0301;
	static constexpr uint16_t DATA_CMD_ID = 0x0101 + ConvertNToCmdIdOffset<N>::value;
	static constexpr size_t LENGTH        = InteractiveHeader::LENGTH + GraphicData::LENGTH * N;

	InteractiveHeader m_Header;
	GraphicData m_GraphicData[N];
};

/**
 * @brief 增加/修改文本
 */
struct ClientGraphicTextModify
{
	static constexpr uint16_t CMD_ID      = 0x0301;
	static constexpr uint16_t DATA_CMD_ID = 0x0110;
	static constexpr size_t LENGTH        = InteractiveHeader::LENGTH + GraphicData::LENGTH + sizeof(uint8_t) * 30;

	InteractiveHeader m_Header;
	GraphicData m_GraphicData;
	uint8_t m_Data[30];
};

// ====================================================================
// 机器人间交互数据 结束
// ====================================================================

/**
 * @brief 裁判系统消息结构
 * 
 * @tparam MessageType 内含消息种类(e.g. ShootData)
 */
template <typename MessageType>
struct RefereeMessage
{
	static constexpr size_t LENGTH = sizeof(FrameHeaderWithCmd) + MessageType::LENGTH + sizeof(FrameTail);

	FrameHeaderWithCmd m_Header;
	MessageType m_Payload;
	FrameTail m_Tail;
};

#pragma pack(pop)

/**
 * @brief Determine whether the model is valid.
 * 
 * @tparam T Model type.
 */
template <typename T>
struct IsValidModel
{
	static constexpr bool value = {sizeof(T) == T::LENGTH};
};

/**
 * @brief Get Nth type of a type sequence.
 * 
 * @tparam N Type index.
 * @tparam Ts Type sequence.
 */
template <int N, typename... Ts>
using NThTypeOf = typename std::tuple_element<N, std::tuple<Ts...>>::type;

/**
	 * @brief Find a type in std::tuple.
	 * 
	 * @tparam T The type to find.
	 * @tparam Tuple The tuple.
	 */
template <typename T, typename Tuple>
struct IndexOf
{
	static_assert(!std::is_same<Tuple, std::tuple<>>::value, "Could not find T in given Tuple");
};

template <typename T, typename... Types>
struct IndexOf<T, std::tuple<T, Types...>>
{
	static constexpr std::size_t value = 0;
};

template <typename T, typename U, typename... Types>
struct IndexOf<T, std::tuple<U, Types...>>
{
	static constexpr std::size_t value = 1 + IndexOf<T, std::tuple<Types...>>::value;
};

/**
 * @brief Count the number of occurrences of the type in std::tuple.
 * 
 * @tparam T Type to count.
 * @tparam Tuple The tuple.
 */
template <typename T, typename Tuple>
struct CountOf
{
};

template <typename T>
struct CountOf<T, std::tuple<>>
{
	static constexpr std::size_t value = 0;
};

template <typename T, typename... Types>
struct CountOf<T, std::tuple<T, Types...>>
{
	static constexpr std::size_t value = 1 + CountOf<T, std::tuple<Types...>>::value;
};

template <typename T, typename U, typename... Types>
struct CountOf<T, std::tuple<U, Types...>>
{
	static constexpr std::size_t value = 0 + CountOf<T, std::tuple<Types...>>::value;
};

class IRefereeBuffer
{
public:
	virtual ~IRefereeBuffer() = default;
	virtual auto PackToNativeBuffer() const -> std::unique_ptr<uint8_t[]> = 0;
	virtual auto Size() const -> size_t = 0;
};

template <typename MessageType>
class RefereeBuffer : public IRefereeBuffer
{
	MessageType m_Msg;

public:
	explicit RefereeBuffer(MessageType& msg)
		: m_Msg(msg)
	{
	}

	auto PackToNativeBuffer() const -> std::unique_ptr<uint8_t[]> override
	{
		static unsigned int idx = 0;
		auto buffer = std::unique_ptr<uint8_t[]>(reinterpret_cast<uint8_t*>(new RefereeMessage<MessageType>()));
		auto* refereeMessage = reinterpret_cast<RefereeMessage<MessageType>*>(buffer.get());
		refereeMessage->m_Header.m_SOF = 0xA5;
		refereeMessage->m_Header.m_CmdId = MessageType::CMD_ID;
		refereeMessage->m_Header.m_DataLength = MessageType::LENGTH;
		refereeMessage->m_Header.m_Seq = idx++; //[TODO]
		DJICRCHelper::AppendCRC8Checksum(buffer.get(), 5);
		refereeMessage->m_Payload = m_Msg;
		DJICRCHelper::AppendCRC16Checksum(buffer.get(), RefereeMessage<MessageType>::LENGTH);
		return buffer;
	}

	auto Size() const -> size_t
	{
		return RefereeMessage<MessageType>::LENGTH;
	}
};

/**
 * @brief Interface of Referee system service.
 */
class IReferee
{
public:
	virtual ~IReferee() = default;
	
	/**
	 * @brief Initialize referee system.
	 * 
	 * @param location The location of referee system (e.g. /dev/ttyTHS2).
	 */
	virtual auto AddReferee(std::string location) -> void = 0;

	virtual auto SendMessage(const IRefereeBuffer& refereeBuffer) const -> void = 0;

	virtual auto Id() const -> uint16_t = 0;

	virtual auto Id(const uint16_t id) -> void = 0;
};

/**
 * @brief The implementation of referee system.
 * 
 * @tparam std::mutex Mutex type (e.g. std::mutex).
 * @tparam MessageTypes The types of message to listen. (e.g. BulletRemain, ShootData).
 */
template <typename Mutex = std::mutex, typename ...MessageTypes>
class Referee : public IReferee, public ossian::IODataBuilder<Mutex, MessageTypes...>
{
public:
	OSSIAN_SERVICE_SETUP(Referee(ossian::UARTManager* uartManager,
		ossian::IOData<MessageTypes>*...listeners))
		: m_UARTManager(uartManager)
		  , m_Id(0)
		  , m_Container(std::make_tuple(listeners...))
	{
	}

	virtual ~Referee() = default;

	auto AddReferee(const std::string location) -> void override
	{
		using namespace ossian::UARTProperties;
		m_RefereeDevice =
			m_UARTManager->AddDevice(location,
			                         115200,
			                         FlowControl::FlowControlNone,
			                         DataBits::DataBits8,
			                         StopBits::StopBits1,
			                         Parity::ParityNone)
			             ->SetCallback(
				             [this](const std::shared_ptr<ossian::BaseDevice>& device,
				                    const size_t length,
				                    const uint8_t* data)
				             {
					             SPDLOG_TRACE("Referee Receive: {}", length);
					             ParseReferee(data, length);
				             });
	}

	auto SendMessage(const IRefereeBuffer& refereeBuffer) const -> void override
	{
		const auto buffer = refereeBuffer.PackToNativeBuffer();
		m_RefereeDevice->WriteRaw(refereeBuffer.Size(), buffer.get());
	}

	auto Id() const -> uint16_t override
	{
		return m_Id;
	}

	auto Id(const uint16_t id) -> void override
	{
		m_Id = id;
	}

private:
	using Container = std::tuple<ossian::IOData<MessageTypes>*...>;
	static_assert((IsValidModel<MessageTypes>::value || ...), "There is a invalid model");
	static_assert(((CountOf<MessageTypes, std::tuple<MessageTypes>>::value == 1) && ...),
		"Redefined message in MessageTypes");

	/**
	 * @brief Try to parse the buffer into specified message type.
	 * 
	 * @tparam MessageType The message type to parse.
	 * @tparam Index The Index of message type in listening sequence.
	 * @param data Buffer to parse.
	 * @param length Buffer's length.
	 * @return size_t The length that have parsed.
	 */
	template <typename MessageType, size_t Index>
	auto ReadData(const uint8_t* data, const size_t length) -> size_t
	{
		const auto cmdId{reinterpret_cast<const FrameHeaderWithCmd*>(data)->m_CmdId};
		if (MessageType::CMD_ID != cmdId)
		{
			return 0;
		}
		if (length < RefereeMessage<MessageType>::LENGTH)
		{
			throw std::runtime_error("Referee: Buffer is incomplete");
		}
		{
			//[TODO]: Check the thread safe rule for the separate lock design
			//std::lock_guard<Mutex> guard{std::get<Index>(m_Mutexes)};
			const auto& msg = *reinterpret_cast<const RefereeMessage<MessageType>*>(data);
			if (!DJICRCHelper::VerifyCRC16Checksum(data, RefereeMessage<MessageType>::LENGTH))
			{
				SPDLOG_ERROR("Referee message verify CRC16 failed. length={}", RefereeMessage<MessageType>::LENGTH);
			}
			if (!DJICRCHelper::VerifyCRC8Checksum(data, sizeof(FrameHeaderWithCmd) - 2))
			{
				SPDLOG_ERROR("Referee message verify CRC8 failed. length={}", sizeof(FrameHeaderWithCmd) - 2);
			}
			std::get<Index>(m_Container)->Set(msg.m_Payload);
		}
		SPDLOG_TRACE("Matched message: {:x}\t Message Length: {}\t Matched: {}",
		             MessageType::CMD_ID, RefereeMessage<MessageType>::LENGTH, Index);
		return RefereeMessage<MessageType>::LENGTH;
	}

	/**
	 * @brief Try to match a message type and parse.
	 * 
	 * @tparam Index 
	 * @param data Buffer.
	 * @param length Buffer's length.
	 * @return size_t The length that have parsed.
	 */
	template <size_t ...Index>
	auto ReadPack(const uint8_t* data, const size_t length, std::index_sequence<Index...>) -> size_t
	{
		try
		{
			auto readLength{(ReadData<NThTypeOf<Index, MessageTypes...>, Index>(data, length) + ...)};
			if (0 == readLength && length > sizeof(FrameHeaderWithCmd))
			{
				readLength = sizeof(FrameHeaderWithCmd)
				             + sizeof(FrameTail)
				             + reinterpret_cast<const FrameHeaderWithCmd*>(data)->m_DataLength;
			}
			return readLength > length ? 0 : readLength;
		}
		catch (std::runtime_error& err)
		{
			SPDLOG_WARN("{}", err.what());
			return 0;
		}
	}

	/**
	 * @brief Parse the whole buffer.
	 * 
	 * @param data Buffer.
	 * @param length Buffer's length.
	 */
	auto ParseReferee(const uint8_t* data, const size_t length)
	{
		size_t remainLength = length;
		while (remainLength > 0)
		{
			auto currentPos{data + (length - remainLength)};
			if (currentPos[0] != 0xA5)
			{
				--remainLength;
				continue;
			}
			//[TODO]: Make sure remainLength won't be a minus value
			auto readLength = ReadPack(currentPos,
			                           remainLength,
			                           std::index_sequence_for<MessageTypes...>());
			remainLength -= readLength > 0 ? readLength : remainLength; ///< 避免因无法匹配进入死循环
		}
	}

	ossian::UARTManager* m_UARTManager;
	std::shared_ptr<ossian::BaseDevice> m_RefereeDevice;
	uint16_t m_Id;
	Container m_Container;
};

template <typename ...MessageTypes>
using RefereeMt = Referee<std::mutex, MessageTypes...>;

template <typename ...MessageTypes>
using RefereeSt = Referee<ossian::null_mutex, MessageTypes...>;

using RefereeAllMessagesMt = RefereeMt<RFIDStatus,
                                       BulletRemain,
                                       ShootData,
                                       DamageStatus,
                                       AerialRobotStatus,
                                       RobotRuneBuff,
                                       RobotPosition,
                                       PowerHeatData,
                                       RobotStatus,
                                       RefereeWarning,
                                       SupplyStation,
                                       VenueEvent,
                                       RobotHP,
                                       MatchStatus,
                                       MatchResult>;

using RefereeAllMessagesSt = RefereeSt<RFIDStatus,
                                       BulletRemain,
                                       ShootData,
                                       DamageStatus,
                                       AerialRobotStatus,
                                       RobotRuneBuff,
                                       RobotPosition,
                                       PowerHeatData,
                                       RobotStatus,
                                       RefereeWarning,
                                       SupplyStation,
                                       VenueEvent,
                                       RobotHP,
                                       MatchStatus,
                                       MatchResult>;

#endif // OSSIAN_REFEREE_HPP
