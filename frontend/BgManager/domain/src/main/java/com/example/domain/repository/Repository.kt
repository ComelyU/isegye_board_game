package com.example.domain.repository

import com.example.domain.model.OrderClass
import com.example.domain.model.TurtleBotClass

interface Repository {
    suspend fun turtleRepo(): Result<TurtleBotClass>

    suspend fun orderRepo(): Result<List<OrderClass>>
}