package com.example.isegyeboard.game_list

import android.content.Context
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.GridLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.isegyeboard.R
import kotlinx.coroutines.launch

class Gamelist : Fragment() {
    private val viewModel: GameViewModel by viewModels()
    private lateinit var gameListRV: RecyclerView
    private lateinit var gameAdapter: GameAdapter
    private lateinit var buttonSearch: ConstraintLayout
    private lateinit var originalGameList: List<GameClass>

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.fragment_gamelist, container, false)
        gameListRV = view.findViewById(R.id.gameListRV)
        buttonSearch = view.findViewById(R.id.buttonSearch)
        return view
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        gameAdapter = GameAdapter(requireContext(), emptyList())

        val sharedPreferences = requireActivity().getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
//        val StoreId = sharedPreferences.getString("StoreId", "")
        val StoreId = "1"

        gameListRV.layoutManager = GridLayoutManager(requireContext(), 2)
        gameListRV.adapter = gameAdapter

        viewModel.getCurrentGameList(StoreId!!)

        lifecycleScope.launch {
            viewModel.gameList.observe(viewLifecycleOwner) { gamelist ->
                gameAdapter.updateData(gamelist) // 데이터 업데이트
                originalGameList = gamelist
            }
        }

    }
}