package com.example.data.mapper.turtle

import com.example.data.model.GameData
import com.example.data.model.OrderData
import com.example.data.model.OrderDetailData
import com.example.data.model.TurtleData
import com.example.domain.model.GameClass
import com.example.domain.model.OrderClass
import com.example.domain.model.OrderDetailClass
import com.example.domain.model.TurtleBotClass

internal fun TurtleData.toDomain() = TurtleBotClass(
    id = id,
    storeId = storeId,
)

internal fun TurtleBotClass.toData() = TurtleData(
    id = id,
    storeId = storeId,
)

internal fun OrderData.toDomain() = OrderClass(
    orderId = orderId,
    customerId = customerId,
    orderStatus = orderStatus,
    orderDetail = orderDetailData.map{ it.toDomain()}
)
internal fun OrderClass.toData() = OrderData(
    orderId = orderId,
    customerId = customerId,
    orderStatus = orderStatus,
    orderDetailData = orderDetail.map { it.toData() }
)

internal fun OrderDetailData.toDomain() = OrderDetailClass(
    orderDetailId = orderDetailId,
    menuName = menuName,
    quantity = quantity,
    totalPrice = totalPrice
)
internal fun OrderDetailClass.toData() = OrderDetailData(
    orderDetailId = orderDetailId,
    menuName = menuName,
    quantity = quantity,
    totalPrice = totalPrice
)


internal fun GameData.toDomain() = GameClass(
    gameOrderId = gameOrderId,
    customerId = customerId,
    gameName = gameName,
    stockLocation = stockLocation,
    orderType = orderType,
    orderStatus = orderStatus,
)
internal fun GameClass.toData() = GameData(
    gameOrderId = gameOrderId,
    customerId = customerId,
    gameName = gameName,
    stockLocation = stockLocation,
    orderType = orderType,
    orderStatus = orderStatus,
)