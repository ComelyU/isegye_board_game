package com.example.data

import com.example.data.model.DeliverData
import com.example.data.model.DeliverResponseData
import com.example.data.model.GameData
import com.example.data.model.OrderData
import com.example.data.model.RemoteResponseData
import com.example.data.model.TurtleData


interface RemoteDataSource {
    suspend fun getTurtleBot(): Result<List<TurtleData>>

    suspend fun getOrderList(): Result<List<OrderData>>

    suspend fun getGameList(): Result<List<GameData>>

    suspend fun startDeliver(deliverData: DeliverData): Result<DeliverResponseData>

    suspend fun cancelGameOrder(gameOrderId: Int) : Result<RemoteResponseData>

    suspend fun startMenuOrder(menuOrderId: Int) : Result<RemoteResponseData>

    suspend fun cancelMenuOrder(menuOrderId: Int) : Result<RemoteResponseData>
}