package com.example.isegyeboard.game_list

import android.content.Context
import android.os.Bundle
import android.view.KeyEvent
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.EditorInfo
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.GridLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.isegyeboard.R
import com.example.isegyeboard.databinding.FragmentGamelistBinding
import com.example.isegyeboard.game_list.model.GameClass
import kotlinx.coroutines.launch

class Gamelist : Fragment() {

    private lateinit var binding: FragmentGamelistBinding
    private val viewModel: GameViewModel by viewModels()
    private lateinit var gameAdapter: GameAdapter
    private lateinit var originalGameList: List<GameClass>

        override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding = FragmentGamelistBinding.inflate(layoutInflater)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        gameAdapter = GameAdapter(requireContext(), emptyList())

        val sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val storeId = sharedPreferences.getString("StoreId", "1")

        binding.gameListRV.adapter = gameAdapter
        binding.gameListRV.layoutManager = GridLayoutManager(requireContext(), 2)

        viewModel.getCurrentGameList(storeId!!)

        lifecycleScope.launch {
            viewModel.gameList.observe(viewLifecycleOwner) { gamelist ->
                gameAdapter.updateData(gamelist) // 데이터 업데이트
                originalGameList = gamelist
            }
        }

        binding.buttonSearch.setOnClickListener{
            performSearch()
        }
        binding.editTextSearch.setOnEditorActionListener { _, actionId, event ->
            if (actionId == EditorInfo.IME_ACTION_SEARCH ||
                (event != null && event.keyCode == KeyEvent.KEYCODE_ENTER)) {
                performSearch()
                true
            } else {
                false
            }
        }

    }

    private fun performSearch() {
        val searchText = binding.editTextSearch.text.toString().trim()
        if (searchText.isNotEmpty()) {
            val filteredList = originalGameList.filter {game ->
                game.game.gameName.contains(searchText, ignoreCase = true)
            }
            gameAdapter.updateData(filteredList)
        } else {
            gameAdapter.updateData(originalGameList)
        }
    }
}