package com.example.domain.usecase

import com.example.domain.model.OrderClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class OrderUseCase @Inject constructor(
    private val repository: Repository
) {
    suspend operator fun invoke(): Result<List<OrderClass>> =
        repository.orderRepo()
}