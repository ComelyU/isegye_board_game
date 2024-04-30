package com.example.data

import com.example.data.model.OrderData
import com.example.data.model.TurtleData


interface RemoteDataSource {
    suspend fun getTurtleBot(): Result<TurtleData>

    suspend fun getOrderList(): Result<List<OrderData>>
}