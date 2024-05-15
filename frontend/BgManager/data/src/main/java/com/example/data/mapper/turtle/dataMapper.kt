package com.example.data.mapper.turtle

import com.example.data.model.DeliverData
import com.example.data.model.DeliverResponseData
import com.example.data.model.GameData
import com.example.data.model.OrderData
import com.example.data.model.OrderDetailData
import com.example.data.model.RemoteResponseData
import com.example.data.model.TurtleData
import com.example.domain.model.DeliverClass
import com.example.domain.model.DeliverResponseClass
import com.example.domain.model.GameClass
import com.example.domain.model.OrderClass
import com.example.domain.model.OrderDetailClass
import com.example.domain.model.RemoteResponseClass
import com.example.domain.model.TurtleClass

internal fun TurtleData.toDomain() = TurtleClass(
    turtleId = turtleId
)

internal fun DeliverClass.toData() = DeliverData(
    turtleId = turtleId,
    orderMenuId = orderMenuId,
    orderGameId = orderGameId,
    returnGameId = returnGameId
)

internal fun DeliverResponseData.toDomain() = DeliverResponseClass(
    status = status
)

internal fun OrderData.toDomain() = OrderClass(
    orderId = orderId,
    customerId = customerId,
    orderStatus = orderStatus,
    roomNumber = roomNumber,
    orderDetail = orderDetailData.map{ it.toDomain()}
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
    roomNumber = roomNumber,
)

internal fun RemoteResponseData.toDomain() = RemoteResponseClass(
    message = message
)