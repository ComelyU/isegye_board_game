package com.example.domain.usecase

import com.example.domain.model.BasicResponseClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class MenuStartUseCase @Inject constructor(
    private val repository: Repository,
) {
    suspend operator fun invoke(menuOrderId: Int): Result<BasicResponseClass> =
        repository.startMenuRepo(menuOrderId)
}