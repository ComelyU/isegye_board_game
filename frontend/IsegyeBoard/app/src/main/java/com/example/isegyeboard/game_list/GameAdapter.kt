package com.example.isegyeboard.game_list

import android.content.Context
import android.graphics.Color
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.navigation.findNavController
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.isegyeboard.R
import com.example.isegyeboard.game_list.model.GameClass
import com.example.isegyeboard.game_list.model.GameTagCategory
import com.example.isegyeboard.game_list.model.StockList
import kotlin.math.ceil

class GameAdapter(private val context: Context, private var dataList: List<GameClass>) :
    RecyclerView.Adapter<GameAdapter.GameViewHolder>() {

    // ViewHolder 클래스 정의
    inner class GameViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val titleTextView: TextView = itemView.findViewById(R.id.listTitle)
        val descriptionTextView: TextView = itemView.findViewById(R.id.listDescription)
        val stockTextView: TextView = itemView.findViewById(R.id.listStock)
        val playerTextView: TextView = itemView.findViewById(R.id.listPlayer)
        val playTimeTextView: TextView = itemView.findViewById(R.id.listPlaytime)
        val difficultyTextView: TextView = itemView.findViewById(R.id.listDifficulty)
        val themeTextView: TextView = itemView.findViewById(R.id.listTheme)
        val imageView: ImageView = itemView.findViewById(R.id.thumbnailImage)

        init {
            itemView.setOnClickListener(this)
        }

        override fun onClick(v: View?) {
            val gameData = dataList[adapterPosition]
            val gameDetail = gameData.game

            if (gameData.isAvailable > 0) {
                val bundle = Bundle().apply {
                    putString("gameId", gameDetail.id.toString())
                    putString("title", gameDetail.gameName)
                    putString("description", gameDetail.gameDetail)
                    putString("thumbnailUrl", gameDetail.gameImgUrl)
                    putString("stock", gameData.isAvailable.toString())
                    putString("minPlayer", gameDetail.minPlayer.toString())
                    putString("maxPlayer", gameDetail.maxPlayer.toString())
                    putString("minPlaytime", gameDetail.minPlaytime.toString())
                    putString("maxPlaytime", gameDetail.maxPlaytime.toString())
                    putString("difficulty", ceil(gameDetail.gameDifficulty).toInt().toString())
                    putString("theme", gameDetail.gameTagCategory.joinToString(", ") { category ->
                        category.codeItemName
                    })
                }
                v?.findNavController()?.navigate(R.id.action_gamelist_to_gamedetail, bundle)
            } else {
                Log.e("List", "e")
            }
        }
    }

    // onCreateViewHolder: ViewHolder 객체를 생성하고 뷰를 연결
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): GameViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.layout_game_item, parent, false)
        return GameViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: GameViewHolder, position: Int) {
        val gameItem = dataList[position]
        val gameItemDetail = gameItem.game
//            println(gameItem.thumbnailUrl)
        if (gameItem.isAvailable == 0) {
            holder.itemView.setBackgroundColor(Color.LTGRAY)
            holder.titleTextView.setTextColor(Color.GRAY)
            holder.descriptionTextView.setTextColor(Color.GRAY)
            holder.playerTextView.setTextColor(Color.GRAY)
            holder.playTimeTextView.setTextColor(Color.GRAY)
            holder.difficultyTextView.setTextColor(Color.GRAY)
            holder.stockTextView.setTextColor(Color.GRAY)
            holder.themeTextView.setTextColor(Color.GRAY)
        } else {
            holder.itemView.setBackgroundColor(Color.WHITE)
        }
        holder.titleTextView.text = gameItemDetail.gameName
        holder.descriptionTextView.text = if (gameItemDetail.gameDetail.length > 30) {
            gameItemDetail.gameDetail.substring(0, 30) + "..."
        } else {
            gameItemDetail.gameDetail
        }
        holder.stockTextView.text = if (gameItem.isAvailable > 0) {
            "재고 : ${gameItem.isAvailable}"
        } else {
            "재고 : X"
        }
        holder.playerTextView.text = if (gameItemDetail.minPlayer == gameItemDetail.maxPlayer) {
            "인원 : ${gameItemDetail.minPlayer}명, "
        } else {
            "인원 : ${gameItemDetail.minPlayer} ~ ${gameItemDetail.maxPlayer}명, "
        }
//            holder.maxPlayerTextView.text = gameItem.maxPlayer.toString()
        holder.playTimeTextView.text = if (gameItemDetail.minPlaytime == gameItemDetail.maxPlaytime) {
            "시간 : ${gameItemDetail.minPlaytime}분, "
        } else {
            "시간 : ${gameItemDetail.minPlaytime} ~ ${gameItemDetail.maxPlaytime}분, "
        }
//            holder.maxPlayTimeTextView.text = gameItem.maxPlaytime.toString()
        holder.difficultyTextView.text = "난이도 : ${"★".repeat(ceil(gameItemDetail.gameDifficulty).toInt())}"

        // 장르리스트
        val tagCategory: List<GameTagCategory> = gameItemDetail.gameTagCategory
        val tagText = tagCategory.joinToString(", ") { category ->
            category.codeItemName
        }
        holder.themeTextView.text = "장르 : $tagText, "

        Glide.with(context)
            .load(gameItemDetail.gameImgUrl)
            .placeholder(R.drawable.ipad) // 로딩이미지
            .error(R.drawable.chess_black) //실패이미지
            .into(holder.imageView)
    }

    // getItemCount: 데이터 목록의 크기 반환
    override fun getItemCount() = dataList.size

    fun updateData(newGameList: List<GameClass>) {
        dataList = newGameList
        notifyDataSetChanged() // 변경 사항을 RecyclerView에 알림
    }
}