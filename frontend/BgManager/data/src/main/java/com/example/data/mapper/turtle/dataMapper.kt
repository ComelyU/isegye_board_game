package com.example.data.mapper.turtle

import com.example.data.model.OrderData
import com.example.data.model.TurtleData
import com.example.domain.model.OrderClass
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
    id = id,
    orderName = orderName,
    quantity = quantity
)
internal fun OrderClass.toData() = OrderData(
    id = id,
    orderName = orderName,
    quantity = quantity
)