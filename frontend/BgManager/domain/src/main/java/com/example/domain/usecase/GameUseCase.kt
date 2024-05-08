package com.example.domain.usecase

import com.example.domain.model.GameClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class GameUseCase @Inject constructor(
    private val repository: Repository
) {
    suspend operator fun invoke(): Result<List<GameClass>> =
        repository.gameRepo()
}