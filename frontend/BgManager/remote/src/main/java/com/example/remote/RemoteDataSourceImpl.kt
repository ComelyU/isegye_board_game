package com.example.remote

import com.example.data.RemoteDataSource
import com.example.data.model.OrderData
import com.example.data.model.TurtleData
import com.example.remote.retrofit.ApiService
import com.google.gson.annotations.SerializedName
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
internal class RemoteDataSourceImpl @Inject constructor(
    private val apiService: ApiService,
) : RemoteDataSource {

    override suspend fun getTurtleBot(): Result<TurtleData> = runCatching {
        val response = apiService.getTurtleBot()
        if (response.isSuccessful) throw Exception()
        TurtleData(
            id = response.body()!!.id,
            storeId = response.body()!!.storeId
        )
    }

    override suspend fun getOrderList(): Result<List<OrderData>> = runCatching {
        val response = apiService.getOrderList()
        if (response.isSuccessful) throw Exception()
        response.body()!!.map { it ->
            OrderData(
                id = it.id,
                orderName = it.orderName,
                quantity = it.quantity
            )
        }
    }
}