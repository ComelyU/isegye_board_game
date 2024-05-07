package com.accio.isegye.store.repository;

import com.accio.isegye.store.dto.RoomResponse;
import com.accio.isegye.store.entity.Room;
import java.util.Arrays;
import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

@Repository
public interface RoomRepository extends JpaRepository<Room, Integer> {

    @Query("select r.fcmToken from Room r where r.id=?1")
    String findFcmTokenById(int id);
    @Query("select r.iotId from Room r where r.id=?1")
    String findIotIdById(int id);

    List<Room> findAllByDeletedAtIsNullAndStoreId(int storeId);

    @Query("select r.id from Room r where r.store.id = ?1 and r.roomNumber = ?2")
    Integer findIdByStoreIdAndRoomNumber(int storeId, int roomNumber);
}
