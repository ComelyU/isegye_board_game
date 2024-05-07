package com.example.isegyeboard.room_history

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.isegyeboard.R

class HistoryAdapter(private val context: Context, private var dataList: List<OrderMenuResponse>) :
    RecyclerView.Adapter<HistoryAdapter.HistoryViewHolder>() {

    // ViewHolder 클래스 정의
    inner class HistoryViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val nameTextView: TextView = itemView.findViewById(R.id.historyItemName)
        val quanTextView: TextView = itemView.findViewById(R.id.historyItemQuan)
        val historyIdTextView: TextView = itemView.findViewById(R.id.historyId)
        val orderStatusTextView: TextView = itemView.findViewById(R.id.orderStatus)

        override fun onClick(v: View?) {
            val history = dataList[adapterPosition]
        }
    }

    // onCreateViewHolder: ViewHolder 객체를 생성하고 뷰를 연결
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): HistoryViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.layout_history_order, parent, false)
        return HistoryViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: HistoryViewHolder, position: Int) {
        val orderMenuResponse = dataList[position]
        val orderMenuDetailList = orderMenuResponse.orderMenuDetail

        val historyId = orderMenuResponse.id
        val orderStatus = orderMenuResponse.orderStatus
        val orderStatusString = if (orderStatus == 0) {
            "주문 접수"
        } else if (orderStatus == 1) {
            "배송 중"
        } else {
            "배송 완료"
        }

        holder.historyIdTextView.append(historyId.toString())
        holder.orderStatusTextView.append(orderStatusString)
        // 주문 메뉴의 상세 정보를 가져와서 표시

        val menuNames = StringBuilder()
        val quantities = StringBuilder()

        for (orderMenuDetail in orderMenuDetailList) {
            menuNames.append("${orderMenuDetail.menuName}\n")
            quantities.append("${orderMenuDetail.quantity} 개\n")
        }

        holder.nameTextView.text = menuNames.toString()
        holder.quanTextView.text = quantities.toString()
    }

    // getItemCount: 데이터 목록의 크기 반환
    override fun getItemCount() = dataList.size

    fun updateData(newHistoryList: List<OrderMenuResponse>) {
        dataList = newHistoryList
        notifyDataSetChanged() // 변경 사항을 RecyclerView에 알림
    }
}
